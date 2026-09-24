#include "qcar2_oampc_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

#include "tf2/exceptions.h"
#include "tf2/time.h"

namespace qcar2_oampc
{
namespace
{
constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kRawMotorSpeedToLinearVelocity =
  (1.0 / (720.0 * 4.0)) * ((13.0 * 19.0) / (70.0 * 37.0)) * (2.0 * kPi) * 0.033;

// Return true when a floating-point value is finite.
bool isFinite(double value)
{
  return std::isfinite(value);
}

// Return the Euclidean distance from the vehicle to the final goal.
double distanceToGoal(
  const VehicleState & state,
  const qcar2_flmpc::Pose2D & goal_pose)
{
  return std::hypot(state.x - goal_pose.x, state.y - goal_pose.y);
}

// Move an angle onto the branch nearest a reference angle.
double unwrapToReference(double angle, double reference_angle)
{
  double unwrapped = angle;
  while (unwrapped - reference_angle > kPi) {
    unwrapped -= 2.0 * kPi;
  }
  while (unwrapped - reference_angle < -kPi) {
    unwrapped += 2.0 * kPi;
  }
  return unwrapped;
}

}  // namespace

// Create ROS interfaces and initialize all OAMPC components.
QCar2OAMPCController::QCar2OAMPCController()
: Node("qcar2_oampc_controller")
{
  declareAndLoadParameters();

  try {
    obstacle_map_.load(obstacle_map_file_);
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      this->get_logger(),
      "Cannot load OAMPC obstacle geometry %s: %s",
      obstacle_map_file_.c_str(),
      exception.what());
    throw;
  }

  result_logger_ = std::make_unique<ResultLogger>(
    enable_result_logging_,
    result_directory_,
    "run_OAMPC_Mk1c_",
    source_config_file_,
    obstacle_map_file_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  try {
    trajectory_generator_ =
      std::make_unique<qcar2_flmpc::QCar2BezierTrajectoryGenerator>(trajectory_parameters_);
    obstacle_activation_manager_ =
      std::make_unique<ObstacleActivationManager>(activation_parameters_);
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(this->get_logger(), "Invalid OAMPC configuration: %s", exception.what());
    throw;
  }

  solver_ = std::make_unique<OAMPCSolver>(oampc_parameters_, obstacle_map_);
  if (!solver_->initialize(this->get_logger())) {
    RCLCPP_FATAL(
      this->get_logger(),
      "Gurobi is mandatory for OAMPC Mk1c and the persistent model could not be initialized.");
    throw std::runtime_error("Gurobi OAMPC initialization failed.");
  }

  command_publisher_ = this->create_publisher<qcar2_interfaces::msg::MotorCommands>(
    command_topic_,
    10);

  rclcpp::QoS path_qos(rclcpp::KeepLast(1));
  path_qos.reliable();
  reference_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
    reference_path_topic_,
    path_qos);
  active_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
    active_path_topic_,
    path_qos);
  publishReferencePreviewPath();

  joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
    joint_state_topic_,
    10,
    std::bind(&QCar2OAMPCController::jointStateCallback, this, std::placeholders::_1));

  goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic_,
    10,
    std::bind(&QCar2OAMPCController::goalCallback, this, std::placeholders::_1));

  const auto period = std::chrono::duration<double>(control_period_);
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&QCar2OAMPCController::controlTimerCallback, this));

  path_publish_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&QCar2OAMPCController::pathPublishTimerCallback, this));

  publishZeroCommand();
  RCLCPP_INFO(
    this->get_logger(),
    "qcar2_oampc_controller loaded %zu obstacle polygons and is waiting for map -> base_link and a /goal_pose trigger.",
    obstacle_map_.obstacles().size());
}

// Read all controller, solver, obstacle, and trajectory parameters.
void QCar2OAMPCController::declareAndLoadParameters()
{
  this->declare_parameter("map_frame", map_frame_);
  this->declare_parameter("base_frame", base_frame_);
  this->declare_parameter("goal_topic", goal_topic_);
  this->declare_parameter("joint_state_topic", joint_state_topic_);
  this->declare_parameter("command_topic", command_topic_);
  this->declare_parameter("reference_path_topic", reference_path_topic_);
  this->declare_parameter("active_path_topic", active_path_topic_);
  this->declare_parameter("result_directory", result_directory_);
  this->declare_parameter("source_config_file", source_config_file_);
  this->declare_parameter("obstacle_map_file", obstacle_map_file_);
  this->declare_parameter("enable_result_logging", enable_result_logging_);
  this->declare_parameter("control_period", control_period_);
  this->declare_parameter("transform_timeout", transform_timeout_);
  this->declare_parameter("feedback_timeout", feedback_timeout_);
  this->declare_parameter("goal_position_tolerance", goal_position_tolerance_);
  this->declare_parameter("stop_on_solver_failure", stop_on_solver_failure_);

  this->declare_parameter("wheelbase", oampc_parameters_.wheelbase);
  this->declare_parameter("epsilon", oampc_parameters_.epsilon);
  this->declare_parameter("Ts", oampc_parameters_.sample_time);
  this->declare_parameter("mpc_N", oampc_parameters_.horizon_steps);
  this->declare_parameter("vx_min", oampc_parameters_.vx_min);
  this->declare_parameter("vx_max", oampc_parameters_.vx_max);
  this->declare_parameter("delta_min", oampc_parameters_.delta_min);
  this->declare_parameter("delta_max", oampc_parameters_.delta_max);
  this->declare_parameter("ax_min", oampc_parameters_.ax_min);
  this->declare_parameter("ax_max", oampc_parameters_.ax_max);
  this->declare_parameter<std::vector<double>>("Q", {120.0, 120.0});
  this->declare_parameter<std::vector<double>>("R", {15.0, 15.0});
  this->declare_parameter("big_M", oampc_parameters_.big_m);
  this->declare_parameter("gamma", oampc_parameters_.gamma);
  this->declare_parameter("slack_weight", oampc_parameters_.slack_weight);
  this->declare_parameter("gurobi_output_flag", oampc_parameters_.gurobi_output_flag);
  this->declare_parameter("gurobi_time_limit", oampc_parameters_.gurobi_time_limit);
  this->declare_parameter("activation_radius", activation_parameters_.activation_radius);
  this->declare_parameter(
    "activation_field_of_view",
    activation_parameters_.field_of_view_degrees);
  this->declare_parameter(
    "max_active_obstacles",
    activation_parameters_.maximum_active_obstacles);

  this->declare_parameter(
    "number_of_waypoints_start",
    trajectory_parameters_.number_of_waypoints_start);
  this->declare_parameter<std::vector<double>>(
    "segment_times_start",
    trajectory_parameters_.segment_times_start);
  this->declare_parameter<std::vector<double>>(
    "waypoints_ref_xy",
    trajectory_parameters_.waypoints_ref_xy);
  this->declare_parameter<std::vector<double>>(
    "segment_times_ref",
    trajectory_parameters_.segment_times_ref);
  this->declare_parameter("theta_start", trajectory_parameters_.theta_start);
  this->declare_parameter("theta_end", trajectory_parameters_.theta_end);
  this->declare_parameter("minimum_segment_time", trajectory_parameters_.minimum_segment_time);
  this->declare_parameter(
    "intermediate_tangent_scale",
    trajectory_parameters_.intermediate_tangent_scale);
  this->declare_parameter("minimum_goal_distance", trajectory_parameters_.minimum_goal_distance);
  this->declare_parameter("zero_endpoint_steering", trajectory_parameters_.zero_endpoint_steering);

  map_frame_ = this->get_parameter("map_frame").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();
  goal_topic_ = this->get_parameter("goal_topic").as_string();
  joint_state_topic_ = this->get_parameter("joint_state_topic").as_string();
  command_topic_ = this->get_parameter("command_topic").as_string();
  reference_path_topic_ = this->get_parameter("reference_path_topic").as_string();
  active_path_topic_ = this->get_parameter("active_path_topic").as_string();
  result_directory_ = this->get_parameter("result_directory").as_string();
  source_config_file_ = this->get_parameter("source_config_file").as_string();
  obstacle_map_file_ = this->get_parameter("obstacle_map_file").as_string();
  enable_result_logging_ = this->get_parameter("enable_result_logging").as_bool();
  control_period_ = this->get_parameter("control_period").as_double();
  transform_timeout_ = this->get_parameter("transform_timeout").as_double();
  feedback_timeout_ = this->get_parameter("feedback_timeout").as_double();
  goal_position_tolerance_ = this->get_parameter("goal_position_tolerance").as_double();
  stop_on_solver_failure_ = this->get_parameter("stop_on_solver_failure").as_bool();

  oampc_parameters_.wheelbase = this->get_parameter("wheelbase").as_double();
  oampc_parameters_.epsilon = this->get_parameter("epsilon").as_double();
  oampc_parameters_.sample_time = this->get_parameter("Ts").as_double();
  oampc_parameters_.horizon_steps = this->get_parameter("mpc_N").as_int();
  oampc_parameters_.vx_min = this->get_parameter("vx_min").as_double();
  oampc_parameters_.vx_max = this->get_parameter("vx_max").as_double();
  oampc_parameters_.delta_min = this->get_parameter("delta_min").as_double();
  oampc_parameters_.delta_max = this->get_parameter("delta_max").as_double();
  oampc_parameters_.ax_min = this->get_parameter("ax_min").as_double();
  oampc_parameters_.ax_max = this->get_parameter("ax_max").as_double();

  const std::vector<double> position_weights = this->get_parameter("Q").as_double_array();
  const std::vector<double> virtual_input_weights = this->get_parameter("R").as_double_array();
  if (position_weights.size() != 2U || virtual_input_weights.size() != 2U) {
    throw std::runtime_error("Q and R must each contain exactly two values.");
  }
  oampc_parameters_.position_weights = {position_weights[0], position_weights[1]};
  oampc_parameters_.virtual_input_weights = {
    virtual_input_weights[0], virtual_input_weights[1]};
  oampc_parameters_.big_m = this->get_parameter("big_M").as_double();
  oampc_parameters_.gamma = this->get_parameter("gamma").as_double();
  oampc_parameters_.slack_weight = this->get_parameter("slack_weight").as_double();
  oampc_parameters_.gurobi_output_flag = this->get_parameter("gurobi_output_flag").as_bool();
  oampc_parameters_.gurobi_time_limit = this->get_parameter("gurobi_time_limit").as_double();
  oampc_parameters_.maximum_active_obstacles =
    this->get_parameter("max_active_obstacles").as_int();

  activation_parameters_.activation_radius =
    this->get_parameter("activation_radius").as_double();
  activation_parameters_.field_of_view_degrees =
    this->get_parameter("activation_field_of_view").as_double();
  activation_parameters_.maximum_active_obstacles =
    oampc_parameters_.maximum_active_obstacles;
  activation_parameters_.safety_margin = oampc_parameters_.gamma;

  trajectory_parameters_.wheelbase = oampc_parameters_.wheelbase;
  trajectory_parameters_.sample_time = oampc_parameters_.sample_time;
  trajectory_parameters_.number_of_waypoints_start =
    this->get_parameter("number_of_waypoints_start").as_int();
  trajectory_parameters_.segment_times_start =
    this->get_parameter("segment_times_start").as_double_array();
  trajectory_parameters_.waypoints_ref_xy =
    this->get_parameter("waypoints_ref_xy").as_double_array();
  trajectory_parameters_.segment_times_ref =
    this->get_parameter("segment_times_ref").as_double_array();
  trajectory_parameters_.theta_start = this->get_parameter("theta_start").as_double();
  trajectory_parameters_.theta_end = this->get_parameter("theta_end").as_double();
  trajectory_parameters_.minimum_segment_time =
    this->get_parameter("minimum_segment_time").as_double();
  trajectory_parameters_.intermediate_tangent_scale =
    this->get_parameter("intermediate_tangent_scale").as_double();
  trajectory_parameters_.minimum_goal_distance =
    this->get_parameter("minimum_goal_distance").as_double();
  trajectory_parameters_.zero_endpoint_steering =
    this->get_parameter("zero_endpoint_steering").as_bool();
}

// Convert raw motor speed feedback into signed longitudinal velocity.
void QCar2OAMPCController::jointStateCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->velocity.empty()) {
    return;
  }

  velocity_feedback_ = msg->velocity[0] * kRawMotorSpeedToLinearVelocity;
  last_velocity_feedback_time_ =
    msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0 ?
    this->get_clock()->now() : rclcpp::Time(msg->header.stamp);
  has_velocity_feedback_ = true;
}

// Use a trigger to create the predefined active trajectory from the current TF pose.
void QCar2OAMPCController::goalCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  (void)msg;
  if (mode_ != ControllerMode::WaitingForGoal) {
    RCLCPP_WARN(
      this->get_logger(),
      "Ignoring 2D Goal trigger because an OAMPC Mk1c run is already active.");
    return;
  }

  publishZeroCommand();
  active_trajectory_ = qcar2_flmpc::ReferenceTrajectory{};
  reference_progress_index_ = 0U;
  solver_->resetWarmStart();
  obstacle_activation_manager_->reset();
  has_active_path_msg_ = false;
  active_path_msg_.poses.clear();

  qcar2_flmpc::Pose2D start_pose;
  if (!readCurrentPoseFromTF(start_pose)) {
    RCLCPP_WARN(
      this->get_logger(),
      "Cannot create the active fixed reference because current TF map -> base_link is unavailable.");
    publishZeroCommand();
    return;
  }

  try {
    active_trajectory_ = trajectory_generator_->generateFullReferenceTrajectory(start_pose);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Fixed-reference trajectory generation failed: %s",
      exception.what());
    publishZeroCommand();
    return;
  }

  if (active_trajectory_.empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Fixed-reference trajectory generation returned an empty trajectory.");
    publishZeroCommand();
    return;
  }

  const auto & final_sample = active_trajectory_.samples.back();
  active_goal_pose_ = qcar2_flmpc::Pose2D{final_sample.x, final_sample.y, final_sample.psi};
  longitudinal_velocity_command_ = 0.0;

  double max_abs_reference_delta = 0.0;
  double max_abs_reference_ax = 0.0;
  for (const auto & sample : active_trajectory_.samples) {
    max_abs_reference_delta = std::max(max_abs_reference_delta, std::abs(sample.delta));
    max_abs_reference_ax = std::max(max_abs_reference_ax, std::abs(sample.ax));
  }
  if (max_abs_reference_delta > std::max(std::abs(oampc_parameters_.delta_min), std::abs(oampc_parameters_.delta_max)) + 1.0e-9) {
    RCLCPP_WARN(
      this->get_logger(),
      "Generated reference exceeds the steering limit: max |delta_ref| = %.4f rad.",
      max_abs_reference_delta);
  }
  if (max_abs_reference_ax > std::max(std::abs(oampc_parameters_.ax_min), std::abs(oampc_parameters_.ax_max)) + 1.0e-9) {
    RCLCPP_WARN(
      this->get_logger(),
      "Generated reference exceeds the acceleration limit: max |ax_ref| = %.4f m/s^2.",
      max_abs_reference_ax);
  }

  if (result_logger_) {
    result_logger_->reset();
    result_logger_->setFullReferenceTrajectory(active_trajectory_);
  }

  tracking_start_time_ = this->get_clock()->now();
  reference_progress_index_ = 0U;
  mode_ = ControllerMode::Tracking;
  publishActivePath();
  RCLCPP_INFO(
    this->get_logger(),
    "2D Goal trigger accepted. Active OAMPC Mk1c trajectory is ready: %zu samples, %.3f s duration. Reference progress follows the projected vehicle position.",
    active_trajectory_.samples.size(),
    active_trajectory_.duration());
}

// Execute one closed-loop OAMPC tracking and avoidance step.
void QCar2OAMPCController::controlTimerCallback()
{
  if (mode_ != ControllerMode::Tracking) {
    publishZeroCommand();
    return;
  }
  if (active_trajectory_.empty()) {
    stopTracking("active fixed-reference trajectory is empty");
    return;
  }

  VehicleState current_state;
  if (!readCurrentVehicleState(current_state)) {
    publishZeroCommand();
    return;
  }

  const rclcpp::Time current_time = this->get_clock()->now();
  const double elapsed_time = (current_time - tracking_start_time_).seconds();
  const std::size_t last_reference_index = active_trajectory_.samples.size() - 1U;
  const std::size_t projected_reference_index = projectPositionToReferencePath(current_state.x, current_state.y);
  reference_progress_index_ = std::max(reference_progress_index_, projected_reference_index);
  const std::size_t reference_index = std::min(reference_progress_index_, last_reference_index);

  const std::size_t near_end_reference_index =
    last_reference_index > static_cast<std::size_t>(oampc_parameters_.horizon_steps) ?
    last_reference_index - static_cast<std::size_t>(oampc_parameters_.horizon_steps) : 0U;
  const bool reference_near_end = reference_index >= near_end_reference_index;
  const double goal_distance = distanceToGoal(current_state, active_goal_pose_);
  if (reference_near_end && goal_distance <= goal_position_tolerance_) {
    stopTracking("fixed PN goal position tolerance reached");
    return;
  }

  if (!solver_->isAvailable()) {
    longitudinal_velocity_command_ = 0.0;
    publishZeroCommand();
    if (stop_on_solver_failure_) {
      stopTracking("Gurobi OAMPC model unavailable");
    }
    return;
  }

  std::vector<std::array<double, 4>> state_reference_horizon;
  std::vector<std::array<double, 2>> input_reference_horizon;
  buildReferenceHorizon(reference_index, state_reference_horizon, input_reference_horizon);

  const double current_psi_for_solver = unwrapToReference(current_state.psi, state_reference_horizon.front()[2]);
  const std::array<double, 4> current_physical_state{{
    current_state.x,
    current_state.y,
    current_psi_for_solver,
    current_state.vx}};

  const std::vector<const Obstacle *> active_obstacles = obstacle_activation_manager_->update(
    obstacle_map_.obstacles(),
    current_state.x,
    current_state.y,
    current_psi_for_solver);

  const auto solve_start_time = std::chrono::steady_clock::now();
  const SolverResult solver_result = solver_->solve(current_physical_state, state_reference_horizon, input_reference_horizon, active_obstacles);
  const auto solve_end_time = std::chrono::steady_clock::now();
  const double solve_time = std::chrono::duration<double>(solve_end_time - solve_start_time).count();
  const auto & log_reference_sample = active_trajectory_.samples[reference_index];

  if (!solver_result.success) {
    if (result_logger_) {
      result_logger_->incrementSolverFailures();
      result_logger_->recordSample(elapsed_time, current_state, log_reference_sample, solver_result, 0.0, solve_time);
    }

    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Gurobi OAMPC solver failed with status %s. Publishing zero command.", solver_result.status_name.c_str());
    longitudinal_velocity_command_ = 0.0;
    publishZeroCommand();
    if (stop_on_solver_failure_) {
      stopTracking("Solver failure !!!");
    }
    return;
  }

  const double steering_angle = clip(solver_result.delta, oampc_parameters_.delta_min,  oampc_parameters_.delta_max);
  const double longitudinal_acceleration = clip(solver_result.ax, oampc_parameters_.ax_min, oampc_parameters_.ax_max);
  longitudinal_velocity_command_ = clip(
    longitudinal_velocity_command_ +
    longitudinal_acceleration * oampc_parameters_.sample_time,
    oampc_parameters_.vx_min,
    oampc_parameters_.vx_max);
  const double longitudinal_velocity_command = longitudinal_velocity_command_;

  // const double longitudinal_velocity_command = clip(current_state.vx + longitudinal_acceleration * oampc_parameters_.sample_time,
  //   oampc_parameters_.vx_min,
  //   oampc_parameters_.vx_max);

  publishMotorCommand(steering_angle, longitudinal_velocity_command, longitudinal_acceleration);

  SolverResult clipped_solver_result = solver_result;
  clipped_solver_result.delta = steering_angle;
  clipped_solver_result.ax = longitudinal_acceleration;
  if (result_logger_) {
    result_logger_->recordSample(elapsed_time, current_state, log_reference_sample, clipped_solver_result, longitudinal_velocity_command, solve_time);
  }

}

// Read the current map to base_link pose without requiring velocity feedback.
bool QCar2OAMPCController::readCurrentPoseFromTF(qcar2_flmpc::Pose2D & pose)
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    (void)transform_timeout_;
    transform = tf_buffer_->lookupTransform(
      map_frame_,
      base_frame_,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & exception) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Waiting for transform %s -> %s: %s",
      map_frame_.c_str(),
      base_frame_.c_str(),
      exception.what());
    return false;
  }

  pose.x = transform.transform.translation.x;
  pose.y = transform.transform.translation.y;
  pose.yaw = yawFromQuaternion(
    transform.transform.rotation.x,
    transform.transform.rotation.y,
    transform.transform.rotation.z,
    transform.transform.rotation.w);
  return isFinite(pose.x) && isFinite(pose.y) && isFinite(pose.yaw);
}

// Read the latest pose and signed longitudinal velocity feedback.
bool QCar2OAMPCController::readCurrentVehicleState(VehicleState & state)
{
  qcar2_flmpc::Pose2D pose;
  if (!readCurrentPoseFromTF(pose)) {
    return false;
  }
  if (!has_velocity_feedback_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Waiting for qcar2_joint velocity feedback.");
    return false;
  }

  const double feedback_age =
    (this->get_clock()->now() - last_velocity_feedback_time_).seconds();
  if (feedback_age > feedback_timeout_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Velocity feedback timeout: %.3f s > %.3f s.",
      feedback_age,
      feedback_timeout_);
    return false;
  }

  state.x = pose.x;
  state.y = pose.y;
  state.psi = pose.yaw;
  state.vx = velocity_feedback_;
  return isFinite(state.x) && isFinite(state.y) &&
         isFinite(state.psi) && isFinite(state.vx);
}

// Project the current vehicle position onto the active reference path.
std::size_t QCar2OAMPCController::projectPositionToReferencePath(double x, double y) const
{
  if (active_trajectory_.samples.empty() || active_trajectory_.samples.size() == 1U) {
    return 0U;
  }

  double best_squared_distance = -1.0;
  std::size_t best_index = 0U;
  for (std::size_t index = 0U; index + 1U < active_trajectory_.samples.size(); ++index) {
    const auto & start = active_trajectory_.samples[index];
    const auto & end = active_trajectory_.samples[index + 1U];
    const double segment_x = end.x - start.x;
    const double segment_y = end.y - start.y;
    const double squared_length = segment_x * segment_x + segment_y * segment_y;

    double projection_ratio = 0.0;
    if (squared_length > 1.0e-12) {
      projection_ratio = clip(((x - start.x) * segment_x + (y - start.y) * segment_y) / squared_length, 0.0, 1.0);
    }

    const double projection_x = start.x + projection_ratio * segment_x;
    const double projection_y = start.y + projection_ratio * segment_y;
    const double error_x = x - projection_x;
    const double error_y = y - projection_y;
    const double squared_distance = error_x * error_x + error_y * error_y;

    if (best_squared_distance < 0.0 || squared_distance < best_squared_distance) {
      best_squared_distance = squared_distance;
      best_index = projection_ratio < 0.5 ? index : index + 1U;
    }
  }

  return std::min(best_index, active_trajectory_.samples.size() - 1U);
}

// Build a reference horizon and repeat the final sample when needed.
void QCar2OAMPCController::buildReferenceHorizon(
  std::size_t start_index,
  std::vector<std::array<double, 4>> & state_reference_horizon,
  std::vector<std::array<double, 2>> & input_reference_horizon) const
{
  state_reference_horizon.clear();
  input_reference_horizon.clear();
  state_reference_horizon.reserve(
    static_cast<std::size_t>(oampc_parameters_.horizon_steps + 1));
  input_reference_horizon.reserve(
    static_cast<std::size_t>(oampc_parameters_.horizon_steps));

  for (int offset = 0; offset <= oampc_parameters_.horizon_steps; ++offset) {
    const std::size_t index = std::min(start_index + static_cast<std::size_t>(offset), active_trajectory_.samples.size() - 1U);
    const auto & sample = active_trajectory_.samples[index];
    state_reference_horizon.push_back({sample.x, sample.y, sample.psi, sample.vx});
  }
  for (int offset = 0; offset < oampc_parameters_.horizon_steps; ++offset) {
    const std::size_t index = std::min(start_index + static_cast<std::size_t>(offset),  active_trajectory_.samples.size() - 1U);
    const auto & sample = active_trajectory_.samples[index];
    input_reference_horizon.push_back({sample.delta, sample.ax});
  }
}

// Build, cache, and publish the fixed reference preview P0 to PN.
void QCar2OAMPCController::publishReferencePreviewPath()
{
  if (!reference_path_publisher_ || !trajectory_generator_) {
    return;
  }

  if (!has_reference_path_msg_) {
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = map_frame_;
    try {
      const auto points = trajectory_generator_->referenceWaypoints();
      if (points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Fixed reference preview path has no waypoints.");
        return;
      }

      const qcar2_flmpc::Pose2D preview_start_pose{
        points.front()[0],
        points.front()[1],
        trajectory_parameters_.theta_start};
      const auto preview_trajectory =
        trajectory_generator_->generateFullReferenceTrajectory(preview_start_pose);
      path_msg.poses.reserve(preview_trajectory.samples.size());
      for (const auto & sample : preview_trajectory.samples) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = map_frame_;
        pose.pose.position.x = sample.x;
        pose.pose.position.y = sample.y;
        pose.pose.position.z = reference_path_z_;
        pose.pose.orientation = quaternionFromYaw(sample.psi);
        path_msg.poses.push_back(pose);
      }
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Cannot build the fixed reference preview path: %s",
        exception.what());
      return;
    }

    reference_path_msg_ = std::move(path_msg);
    has_reference_path_msg_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "Cached fixed sampled reference path P0 -> PN on %s with %zu poses.",
      reference_path_topic_.c_str(),
      reference_path_msg_.poses.size());
  }

  const auto stamp = this->get_clock()->now();
  reference_path_msg_.header.stamp = stamp;
  reference_path_msg_.header.frame_id = map_frame_;
  for (auto & pose : reference_path_msg_.poses) {
    pose.header = reference_path_msg_.header;
  }
  reference_path_publisher_->publish(reference_path_msg_);
}

// Build, cache, and publish the active trajectory S0 to P0 to PN.
void QCar2OAMPCController::publishActivePath()
{
  if (!active_path_publisher_ || active_trajectory_.empty()) {
    return;
  }

  if (!has_active_path_msg_) {
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = map_frame_;
    path_msg.poses.reserve(active_trajectory_.samples.size());
    for (const auto & sample : active_trajectory_.samples) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = map_frame_;
      pose.pose.position.x = sample.x;
      pose.pose.position.y = sample.y;
      pose.pose.position.z = active_path_z_;
      pose.pose.orientation = quaternionFromYaw(sample.psi);
      path_msg.poses.push_back(pose);
    }

    active_path_msg_ = std::move(path_msg);
    has_active_path_msg_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "Cached active path S0 -> ... -> P0 -> ... -> PN on %s with %zu poses.",
      active_path_topic_.c_str(),
      active_path_msg_.poses.size());
  }

  const auto stamp = this->get_clock()->now();
  active_path_msg_.header.stamp = stamp;
  active_path_msg_.header.frame_id = map_frame_;
  for (auto & pose : active_path_msg_.poses) {
    pose.header = active_path_msg_.header;
  }
  active_path_publisher_->publish(active_path_msg_);
}

// Republish cached paths so RViz2 can attach after launch or after the trigger.
void QCar2OAMPCController::pathPublishTimerCallback()
{
  if (has_reference_path_msg_ && reference_path_publisher_ &&
    reference_path_publisher_->get_subscription_count() > 0)
  {
    publishReferencePreviewPath();
  }
  if (has_active_path_msg_ && active_path_publisher_ &&
    active_path_publisher_->get_subscription_count() > 0)
  {
    publishActivePath();
  }
}

// Send zero steering, speed, and acceleration commands.
void QCar2OAMPCController::publishZeroCommand()
{
  publishMotorCommand(0.0, 0.0, 0.0);
}

// Publish one command through the unchanged hardware interface.
void QCar2OAMPCController::publishMotorCommand(
  double steering_angle,
  double longitudinal_velocity_command,
  double longitudinal_acceleration)
{
  qcar2_interfaces::msg::MotorCommands command;
  command.motor_names = {"steering_angle", "desired_speed", "desired_acceleration"};
  command.values = {
    steering_angle,
    longitudinal_velocity_command,
    longitudinal_acceleration};
  command_publisher_->publish(command);
}

// Stop tracking, export results, and reset online solver state.
void QCar2OAMPCController::stopTracking(const std::string & reason)
{
  publishZeroCommand();
  reference_progress_index_ = 0U;
  longitudinal_velocity_command_ = 0.0;
  if (solver_) {
    solver_->resetWarmStart();
  }
  if (obstacle_activation_manager_) {
    obstacle_activation_manager_->reset();
  }
  if (result_logger_) {
    result_logger_->exportRunResults(reason, this->get_logger());
  }
  mode_ = ControllerMode::WaitingForGoal;
  RCLCPP_INFO(
    this->get_logger(),
    "OAMPC Mk1c tracking stopped: %s. Waiting for a new 2D Goal trigger.",
    reason.c_str());
}

// Convert a quaternion into planar yaw.
double QCar2OAMPCController::yawFromQuaternion(
  double x,
  double y,
  double z,
  double w)
{
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

// Convert planar yaw into a quaternion.
geometry_msgs::msg::Quaternion QCar2OAMPCController::quaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = std::sin(0.5 * yaw);
  quaternion.w = std::cos(0.5 * yaw);
  return quaternion;
}

// Clip one scalar to the provided closed interval.
double QCar2OAMPCController::clip(
  double value,
  double lower_bound,
  double upper_bound)
{
  return std::min(std::max(value, lower_bound), upper_bound);
}

}  // namespace qcar2_oampc

// Start the physical QCar2 OAMPC controller node.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<qcar2_oampc::QCar2OAMPCController>());
  rclcpp::shutdown();
  return 0;
}
