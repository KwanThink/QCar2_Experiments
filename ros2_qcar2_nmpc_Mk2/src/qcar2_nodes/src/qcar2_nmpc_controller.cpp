#include "qcar2_nmpc_controller.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

#include "tf2/exceptions.h"
#include "tf2/time.h"

#ifdef QCAR2_HAS_ACADOS_SOLVER
extern "C" {
#include "acados_solver_qcar2_single_track.h"
#include "acados_c/ocp_nlp_interface.h"
}
#endif

namespace qcar2_nmpc
{
namespace
{
constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kRawMotorSpeedToLinearVelocity =  (1.0 / (720.0 * 4.0)) * ((13.0 * 19.0) / (70.0 * 37.0)) * (2.0 * kPi) * 0.033;

// Return true when a floating-point value can safely be used by the controller.
bool isFinite(double value)
{
  return std::isfinite(value);
}

// Return the Euclidean distance between the vehicle state and the active goal position.
double distanceToGoal(const VehicleState & state, const Pose2D & goal_pose)
{
  return std::hypot(state.x - goal_pose.x, state.y - goal_pose.y);
}

// Shift angle by multiples of 2*pi so it is represented on the same branch
// as reference_angle. This avoids a false jump between +pi and -pi.
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

// Return the absolute heading error between the vehicle and goal on the shortest angle branch.
double yawErrorToGoal(const VehicleState & state, const Pose2D & goal_pose)
{
  return std::abs(unwrapToReference(state.psi, goal_pose.yaw) - goal_pose.yaw);
}

// Keep result folder names safe and predictable when shape_type comes from YAML.
// Allowed characters are [A-Za-z0-9_-]; spaces and other separators become '_'.
std::string sanitizeRunDirectoryToken(const std::string & raw_token)
{
  std::string token;
  token.reserve(raw_token.size());

  bool previous_was_separator = false;
  for (const unsigned char character : raw_token) {
    if (std::isalnum(character)) {
      token.push_back(static_cast<char>(character));
      previous_was_separator = false;
    } else if (character == '_' || character == '-') {
      if (!token.empty() && !previous_was_separator) {
        token.push_back(static_cast<char>(character));
        previous_was_separator = true;
      }
    } else if (!token.empty() && !previous_was_separator) {
      token.push_back('_');
      previous_was_separator = true;
    }
  }

  while (!token.empty() && (token.back() == '_' || token.back() == '-')) {
    token.pop_back();
  }

  return token.empty() ? std::string("FixedRef") : token;
}

}  // namespace
// Store the NMPC dimensions and constraints used by the generated solver.
AcadosSingleTrackSolver::AcadosSingleTrackSolver(const NMPCParameters & parameters)
: parameters_(parameters)
{
}

// Release the generated acados solver resources.
AcadosSingleTrackSolver::~AcadosSingleTrackSolver()
{
#ifdef QCAR2_HAS_ACADOS_SOLVER
  if (solver_capsule_ != nullptr) {
    auto * typed_capsule = static_cast<qcar2_single_track_solver_capsule *>(solver_capsule_);
    qcar2_single_track_acados_free(typed_capsule);
    qcar2_single_track_acados_free_capsule(typed_capsule);
    solver_capsule_ = nullptr;
  }
#endif
}

// Create and initialize the generated acados solver capsule.
bool AcadosSingleTrackSolver::initialize(const rclcpp::Logger & logger)
{
#ifdef QCAR2_HAS_ACADOS_SOLVER
  auto * typed_capsule = qcar2_single_track_acados_create_capsule();
  if (typed_capsule == nullptr) {
    RCLCPP_ERROR(logger, "Failed to allocate qcar2_single_track acados capsule.");
    solver_available_ = false;
    return false;
  }

  const int status = qcar2_single_track_acados_create(typed_capsule);
  if (status != 0) {
    RCLCPP_ERROR(logger, "qcar2_single_track_acados_create failed with status %d.", status);
    qcar2_single_track_acados_free_capsule(typed_capsule);
    solver_available_ = false;
    return false;
  }

  solver_capsule_ = typed_capsule;
  solver_available_ = true;
  RCLCPP_INFO(logger, "Generated acados solver is available for qcar2_nmpc_controller.");
  return true;
#else
  (void)logger;
  solver_available_ = false;
  return false;
#endif
}

// Solve one NMPC step and reuse the shifted previous optimum as the initial guess.
SolverResult AcadosSingleTrackSolver::solve(
  const std::array<double, 4> & current_state,
  const std::vector<std::array<double, 4>> & state_reference_horizon,
  const std::vector<std::array<double, 2>> & input_reference_horizon,
  const std::array<double, 2> & input_initial_guess)
{
  SolverResult result;

#ifndef QCAR2_HAS_ACADOS_SOLVER
  (void)current_state;
  (void)state_reference_horizon;
  (void)input_reference_horizon;
  (void)input_initial_guess;
  result.status = -99;
  result.success = false;
  return result;
#else
  if (!solver_available_ || solver_capsule_ == nullptr) {
    result.status = -98;
    result.success = false;
    return result;
  }

  const std::size_t horizon_steps = static_cast<std::size_t>(parameters_.horizon_steps);
  if (parameters_.horizon_steps <= 0 ||
    state_reference_horizon.size() != horizon_steps + 1U ||
    input_reference_horizon.size() != horizon_steps)
  {
    resetWarmStart();
    result.status = -97;
    result.success = false;
    return result;
  }

  auto * capsule = static_cast<qcar2_single_track_solver_capsule *>(solver_capsule_);
  ocp_nlp_config * nlp_config = qcar2_single_track_acados_get_nlp_config(capsule);
  ocp_nlp_dims * nlp_dims = qcar2_single_track_acados_get_nlp_dims(capsule);
  ocp_nlp_in * nlp_in = qcar2_single_track_acados_get_nlp_in(capsule);
  ocp_nlp_out * nlp_out = qcar2_single_track_acados_get_nlp_out(capsule);

  // Fix the first predicted state to the latest measured vehicle state.
  double x0[4] = {current_state[0], current_state[1], current_state[2], current_state[3]};
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", x0);
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", x0);

  const bool use_warm_start =
    has_warm_start_ &&
    previous_state_solution_.size() == horizon_steps + 1U &&
    previous_input_solution_.size() == horizon_steps;

  for (int stage = 0; stage < parameters_.horizon_steps; ++stage) {
    // Set the stage reference used by the nonlinear least-squares cost.
    double yref[6] = {
      state_reference_horizon[stage][0],
      state_reference_horizon[stage][1],
      state_reference_horizon[stage][2],
      state_reference_horizon[stage][3],
      input_reference_horizon[stage][0],
      input_reference_horizon[stage][1]};
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, stage, "yref", yref);

    // Shift the previous optimum by one step, or fall back to the current reference.
    std::array<double, 4> state_guess = state_reference_horizon[stage];
    std::array<double, 2> input_guess = input_reference_horizon[stage];
    if (use_warm_start) {
      const std::size_t shifted_state_index = std::min(
        static_cast<std::size_t>(stage + 1), horizon_steps);
      const std::size_t shifted_input_index = std::min(
        static_cast<std::size_t>(stage + 1), horizon_steps - 1U);
      state_guess = previous_state_solution_[shifted_state_index];
      input_guess = previous_input_solution_[shifted_input_index];
    } else if (stage == 0) {
      input_guess = input_initial_guess;
    }

    if (stage == 0) {
      state_guess = current_state;
    }

    double x_guess[4] = {
      state_guess[0], state_guess[1], state_guess[2], state_guess[3]};
    double u_guess[2] = {input_guess[0], input_guess[1]};
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, stage, "x", x_guess);
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, stage, "u", u_guess);
  }

  // Set the terminal reference and the shifted terminal state guess.
  double yref_terminal[4] = {
    state_reference_horizon[horizon_steps][0],
    state_reference_horizon[horizon_steps][1],
    state_reference_horizon[horizon_steps][2],
    state_reference_horizon[horizon_steps][3]};
  ocp_nlp_cost_model_set(
    nlp_config, nlp_dims, nlp_in, parameters_.horizon_steps, "yref", yref_terminal);

  const std::array<double, 4> terminal_state_guess = use_warm_start ?
    previous_state_solution_.back() : state_reference_horizon.back();
  double x_terminal_guess[4] = {
    terminal_state_guess[0],
    terminal_state_guess[1],
    terminal_state_guess[2],
    terminal_state_guess[3]};
  ocp_nlp_out_set(
    nlp_config, nlp_dims, nlp_out, nlp_in,
    parameters_.horizon_steps, "x", x_terminal_guess);

  result.status = qcar2_single_track_acados_solve(capsule);
  if (result.status != 0) {
    resetWarmStart();
    result.success = false;
    return result;
  }

  double optimal_input[2] = {0.0, 0.0};
  ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", optimal_input);
  if (!isFinite(optimal_input[0]) || !isFinite(optimal_input[1])) {
    resetWarmStart();
    result.status = -96;
    result.success = false;
    return result;
  }

  // Store the complete optimal state and input trajectories for the next control step.
  std::vector<std::array<double, 4>> state_solution(horizon_steps + 1U);
  std::vector<std::array<double, 2>> input_solution(horizon_steps);
  bool solution_is_finite = true;

  for (int stage = 0; stage <= parameters_.horizon_steps; ++stage) {
    double optimal_state[4] = {0.0, 0.0, 0.0, 0.0};
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, stage, "x", optimal_state);
    state_solution[stage] = {
      optimal_state[0], optimal_state[1], optimal_state[2], optimal_state[3]};
    solution_is_finite = solution_is_finite &&
      isFinite(optimal_state[0]) && isFinite(optimal_state[1]) &&
      isFinite(optimal_state[2]) && isFinite(optimal_state[3]);
  }

  for (int stage = 0; stage < parameters_.horizon_steps; ++stage) {
    double optimal_stage_input[2] = {0.0, 0.0};
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, stage, "u", optimal_stage_input);
    input_solution[stage] = {optimal_stage_input[0], optimal_stage_input[1]};
    solution_is_finite = solution_is_finite &&
      isFinite(optimal_stage_input[0]) && isFinite(optimal_stage_input[1]);
  }

  if (solution_is_finite) {
    previous_state_solution_ = std::move(state_solution);
    previous_input_solution_ = std::move(input_solution);
    has_warm_start_ = true;
  } else {
    resetWarmStart();
  }

  result.delta = optimal_input[0];
  result.ax = optimal_input[1];
  result.success = true;
  return result;
#endif
}

// Clear the stored state and input trajectories used for warm-starting.
void AcadosSingleTrackSolver::resetWarmStart()
{
  previous_state_solution_.clear();
  previous_input_solution_.clear();
  has_warm_start_ = false;
}

// Create ROS interfaces and initialize the fixed-reference NMPC controller.
QCar2NMPCController::QCar2NMPCController(): Node("qcar2_nmpc_controller")
{
  declareAndLoadParameters();
  const std::string sanitized_shape_type = sanitizeRunDirectoryToken(shape_type_);
  result_logger_ = std::make_unique<ResultLogger>(
    enable_result_logging_,
    result_directory_,
    "run_Mk2_" + sanitized_shape_type + "_",
    source_config_file_);
  if (sanitized_shape_type != shape_type_) {
    RCLCPP_WARN(
      this->get_logger(),
      "Result shape_type '%s' contains characters that are not safe for folder names. Using '%s' instead.",
      shape_type_.c_str(),
      sanitized_shape_type.c_str());
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  try {
    trajectory_generator_ = std::make_unique<QCar2BezierTrajectoryGenerator>(trajectory_parameters_);
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(this->get_logger(), "Invalid fixed-reference trajectory configuration: %s", exception.what());
    throw;
  }

  solver_ = std::make_unique<AcadosSingleTrackSolver>(nmpc_parameters_);
  if (!solver_->initialize(this->get_logger())) {
    RCLCPP_WARN(
      this->get_logger(),
      "Generated acados solver was not found at build time. The controller will stay safe with zero commands until the solver is generated and the package is rebuilt.");
  }

  command_publisher_ = this->create_publisher<qcar2_interfaces::msg::MotorCommands>(command_topic_, 10);

  // Use normal reliable QoS instead of transient_local to keep RViz visualization light.
  // The timer republishes cached path messages, so RViz still sees them when displays
  // are added after launch or after the 2D Goal trigger.
  rclcpp::QoS path_qos(rclcpp::KeepLast(1));
  path_qos.reliable();
  reference_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(reference_path_topic_, path_qos);
  active_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(active_path_topic_, path_qos);
  publishReferencePreviewPath();

  joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
    joint_state_topic_, 10, std::bind(&QCar2NMPCController::jointStateCallback, this, std::placeholders::_1));

  goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic_, 10, std::bind(&QCar2NMPCController::goalCallback, this, std::placeholders::_1));

  const auto period = std::chrono::duration<double>(control_period_);
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&QCar2NMPCController::controlTimerCallback, this));

  path_publish_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&QCar2NMPCController::pathPublishTimerCallback, this));

  publishZeroCommand();
  RCLCPP_INFO(
    this->get_logger(),
    "qcar2_nmpc_controller loaded fixed reference path and is waiting for map -> base_link and /goal_pose trigger.");
}

// Declare and load controller, model, logging, and fixed-reference parameters.
void QCar2NMPCController::declareAndLoadParameters()
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
  this->declare_parameter("enable_result_logging", enable_result_logging_);
  this->declare_parameter("shape_type", shape_type_);
  this->declare_parameter("control_period", control_period_);
  this->declare_parameter("transform_timeout", transform_timeout_);
  this->declare_parameter("feedback_timeout", feedback_timeout_);
  this->declare_parameter("goal_position_tolerance", goal_position_tolerance_);
  this->declare_parameter("goal_yaw_tolerance", goal_yaw_tolerance_);
  this->declare_parameter("stop_on_solver_failure", stop_on_solver_failure_);

  this->declare_parameter("wheelbase", nmpc_parameters_.wheelbase);
  this->declare_parameter("Ts", nmpc_parameters_.sample_time);
  this->declare_parameter("mpc_N", nmpc_parameters_.horizon_steps);
  this->declare_parameter("vx_min", nmpc_parameters_.vx_min);
  this->declare_parameter("vx_max", nmpc_parameters_.vx_max);
  this->declare_parameter("delta_min", nmpc_parameters_.delta_min);
  this->declare_parameter("delta_max", nmpc_parameters_.delta_max);
  this->declare_parameter("ax_min", nmpc_parameters_.ax_min);
  this->declare_parameter("ax_max", nmpc_parameters_.ax_max);
  this->declare_parameter<std::vector<double>>("Q", {25.0, 25.0, 8.0, 3.0});
  this->declare_parameter<std::vector<double>>("R", {0.4, 0.4});
  this->declare_parameter<std::vector<double>>("Qe", {60.0, 60.0, 15.0, 6.0});

  this->declare_parameter("number_of_waypoints_start", trajectory_parameters_.number_of_waypoints_start);
  this->declare_parameter<std::vector<double>>("segment_times_start", trajectory_parameters_.segment_times_start);
  this->declare_parameter<std::vector<double>>("waypoints_ref_xy", trajectory_parameters_.waypoints_ref_xy);
  this->declare_parameter<std::vector<double>>("segment_times_ref", trajectory_parameters_.segment_times_ref);
  this->declare_parameter("theta_start", trajectory_parameters_.theta_start);
  this->declare_parameter("theta_end", trajectory_parameters_.theta_end);
  this->declare_parameter("minimum_segment_time", trajectory_parameters_.minimum_segment_time);
  this->declare_parameter("intermediate_tangent_scale", trajectory_parameters_.intermediate_tangent_scale);
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
  enable_result_logging_ = this->get_parameter("enable_result_logging").as_bool();
  shape_type_ = this->get_parameter("shape_type").as_string();
  control_period_ = this->get_parameter("control_period").as_double();
  transform_timeout_ = this->get_parameter("transform_timeout").as_double();
  feedback_timeout_ = this->get_parameter("feedback_timeout").as_double();
  goal_position_tolerance_ = this->get_parameter("goal_position_tolerance").as_double();
  goal_yaw_tolerance_ = this->get_parameter("goal_yaw_tolerance").as_double();
  stop_on_solver_failure_ = this->get_parameter("stop_on_solver_failure").as_bool();

  nmpc_parameters_.wheelbase = this->get_parameter("wheelbase").as_double();
  nmpc_parameters_.sample_time = this->get_parameter("Ts").as_double();
  nmpc_parameters_.horizon_steps = this->get_parameter("mpc_N").as_int();
  nmpc_parameters_.vx_min = this->get_parameter("vx_min").as_double();
  nmpc_parameters_.vx_max = this->get_parameter("vx_max").as_double();
  nmpc_parameters_.delta_min = this->get_parameter("delta_min").as_double();
  nmpc_parameters_.delta_max = this->get_parameter("delta_max").as_double();
  nmpc_parameters_.ax_min = this->get_parameter("ax_min").as_double();
  nmpc_parameters_.ax_max = this->get_parameter("ax_max").as_double();

  trajectory_parameters_.wheelbase = nmpc_parameters_.wheelbase;
  trajectory_parameters_.sample_time = nmpc_parameters_.sample_time;
  trajectory_parameters_.number_of_waypoints_start = this->get_parameter("number_of_waypoints_start").as_int();
  trajectory_parameters_.segment_times_start = this->get_parameter("segment_times_start").as_double_array();
  trajectory_parameters_.waypoints_ref_xy = this->get_parameter("waypoints_ref_xy").as_double_array();
  trajectory_parameters_.segment_times_ref = this->get_parameter("segment_times_ref").as_double_array();
  trajectory_parameters_.theta_start = this->get_parameter("theta_start").as_double();
  trajectory_parameters_.theta_end = this->get_parameter("theta_end").as_double();
  trajectory_parameters_.minimum_segment_time = this->get_parameter("minimum_segment_time").as_double();
  trajectory_parameters_.intermediate_tangent_scale = this->get_parameter("intermediate_tangent_scale").as_double();
  trajectory_parameters_.minimum_goal_distance = this->get_parameter("minimum_goal_distance").as_double();
  trajectory_parameters_.zero_endpoint_steering = this->get_parameter("zero_endpoint_steering").as_bool();
}

// Convert raw motor speed feedback into longitudinal vehicle velocity.
void QCar2NMPCController::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->velocity.empty()) {
    return;
  }

  velocity_feedback_ = msg->velocity[0] * kRawMotorSpeedToLinearVelocity;
  last_velocity_feedback_time_ = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0 ?
    this->get_clock()->now() : rclcpp::Time(msg->header.stamp);
  has_velocity_feedback_ = true;
}

// Use a received 2D Goal as the trigger for a new fixed-reference run.
void QCar2NMPCController::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  (void)msg;  // RViz2 2D Goal is only a trigger. Its x/y/yaw are intentionally ignored.

  publishZeroCommand();
  mode_ = ControllerMode::WaitingForGoal;
  solver_->resetWarmStart();
  active_trajectory_.samples.clear();
  has_active_path_msg_ = false;
  active_path_msg_.poses.clear();

  Pose2D start_pose;
  if (!readCurrentPoseFromTF(start_pose)) {
    RCLCPP_WARN(this->get_logger(), "Cannot create active reference because current TF map -> base_link is unavailable.");
    publishZeroCommand();
    return;
  }

  try {
    active_trajectory_ = trajectory_generator_->generateFullReferenceTrajectory(start_pose);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(this->get_logger(), "Fixed-reference trajectory generation failed: %s", exception.what());
    publishZeroCommand();
    return;
  }

  if (active_trajectory_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Fixed-reference trajectory generation returned an empty trajectory.");
    publishZeroCommand();
    return;
  }

  const auto & final_sample = active_trajectory_.samples.back();
  active_goal_pose_ = Pose2D{final_sample.x, final_sample.y, final_sample.psi};
  last_control_input_ = {0.0, 0.0};
  longitudinal_velocity_command_ = 0.0;

  if (result_logger_) {
    result_logger_->reset();
  }
  tracking_start_time_ = this->get_clock()->now();
  mode_ = ControllerMode::Tracking;

  publishActivePath();
  RCLCPP_INFO(
    this->get_logger(),
    "2D Goal trigger accepted. Active fixed-reference trajectory is ready: %zu samples, %.3f s duration. Tracking starts now.",
    active_trajectory_.samples.size(),
    active_trajectory_.duration());
}

// Execute one closed-loop NMPC tracking step.
void QCar2NMPCController::controlTimerCallback()
{
  if (mode_ != ControllerMode::Tracking) {
    publishZeroCommand();
    return;
  }

  if (active_trajectory_.empty()) {
    stopTracking("active trajectory is empty");
    return;
  }

  VehicleState current_state;
  if (!readCurrentVehicleState(current_state)) {
    publishZeroCommand();
    return;
  }

  const double elapsed_time = (this->get_clock()->now() - tracking_start_time_).seconds();
  std::size_t reference_index = static_cast<std::size_t>(
    std::max(0.0, std::round(elapsed_time / nmpc_parameters_.sample_time)));
  reference_index = std::min(reference_index, active_trajectory_.samples.size() - 1);

  // Keep the final reference active and only stop after reaching the fixed PN goal pose.
  const std::size_t horizon_steps = static_cast<std::size_t>(nmpc_parameters_.horizon_steps);
  const std::size_t near_end_threshold = active_trajectory_.samples.size() > horizon_steps + 1U ?
    active_trajectory_.samples.size() - horizon_steps - 1U : 0U;
  const bool near_end_of_reference = reference_index >= near_end_threshold;
  const double goal_distance = distanceToGoal(current_state, active_goal_pose_);
  const double goal_yaw_error = yawErrorToGoal(current_state, active_goal_pose_);

  if (near_end_of_reference &&
    goal_distance <= goal_position_tolerance_ &&
    goal_yaw_error <= goal_yaw_tolerance_)
  {
    stopTracking("fixed PN goal pose tolerance reached");
    return;
  }

  if (!solver_->isAvailable()) {
    publishZeroCommand();
    return;
  }

  std::vector<std::array<double, 4>> state_reference_horizon;
  std::vector<std::array<double, 2>> input_reference_horizon;
  buildReferenceHorizon(reference_index, state_reference_horizon, input_reference_horizon);

  const double current_psi_for_solver = unwrapToReference(
    current_state.psi,
    state_reference_horizon.front()[2]);

  const std::array<double, 4> current_state_array{{
    current_state.x,
    current_state.y,
    current_psi_for_solver,
    current_state.vx}};

  const auto solve_start_time = std::chrono::steady_clock::now();
  const SolverResult solver_result = solver_->solve(
    current_state_array,
    state_reference_horizon,
    input_reference_horizon,
    last_control_input_);
  const auto solve_end_time = std::chrono::steady_clock::now();
  const double solve_time = std::chrono::duration<double>(solve_end_time - solve_start_time).count();

  const std::size_t log_reference_index = std::min(reference_index, active_trajectory_.samples.size() - 1);
  const auto & log_reference_sample = active_trajectory_.samples[log_reference_index];

  if (!solver_result.success) {
    if (result_logger_) {
      result_logger_->incrementSolverFailures();
      result_logger_->recordSample(elapsed_time, current_state, log_reference_sample, solver_result, 0.0, solve_time);
    }
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "acados solver failed with status %d. Publishing zero command.",
      solver_result.status);
    publishZeroCommand();
    if (stop_on_solver_failure_) {
      stopTracking("Solver failure !!!");
    }
    return;
  }

  // Packaging optimal control input
  const double steering_angle = clip(
    solver_result.delta,
    nmpc_parameters_.delta_min,
    nmpc_parameters_.delta_max);

  const double longitudinal_acceleration = clip(
    solver_result.ax,
    nmpc_parameters_.ax_min,
    nmpc_parameters_.ax_max);

  // One-step vx command generated from current measured vx and optimal ax
  longitudinal_velocity_command_ = clip(
    longitudinal_velocity_command_ + longitudinal_acceleration * nmpc_parameters_.sample_time,
    nmpc_parameters_.vx_min,
    nmpc_parameters_.vx_max);

  const double longitudinal_velocity_command = longitudinal_velocity_command_;

  publishMotorCommand(
    steering_angle,
    longitudinal_velocity_command,
    longitudinal_acceleration);

  SolverResult clipped_solver_result = solver_result;
  clipped_solver_result.delta = steering_angle;
  clipped_solver_result.ax = longitudinal_acceleration;
  if (result_logger_) {
    result_logger_->recordSample(
      elapsed_time,
      current_state,
      log_reference_sample,
      clipped_solver_result,
      longitudinal_velocity_command,
      solve_time);
  }

  last_control_input_ = {steering_angle, longitudinal_acceleration};
}

// Read the current planar vehicle pose from map to base_link TF.
bool QCar2NMPCController::readCurrentPoseFromTF(Pose2D & pose)
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    (void)transform_timeout_;
    transform = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & exception) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Waiting for transform %s -> %s: %s", map_frame_.c_str(), base_frame_.c_str(), exception.what());
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

// Combine the latest TF pose and velocity feedback into the measured state.
bool QCar2NMPCController::readCurrentVehicleState(VehicleState & state)
{
  Pose2D pose;
  if (!readCurrentPoseFromTF(pose)) {
    return false;
  }

  if (!has_velocity_feedback_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for qcar2_joint velocity feedback.");
    return false;
  }

  const double feedback_age = (this->get_clock()->now() - last_velocity_feedback_time_).seconds();
  if (feedback_age > feedback_timeout_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Velocity feedback timeout: %.3f s > %.3f s.", feedback_age, feedback_timeout_);
    return false;
  }

  state.x = pose.x;
  state.y = pose.y;
  state.psi = pose.yaw;
  state.vx = velocity_feedback_;

  return isFinite(state.x) && isFinite(state.y) && isFinite(state.psi) && isFinite(state.vx);
}

// Build state and input references while holding the final sample beyond the path end.
void QCar2NMPCController::buildReferenceHorizon(
  std::size_t start_index,
  std::vector<std::array<double, 4>> & state_reference_horizon,
  std::vector<std::array<double, 2>> & input_reference_horizon) const
{
  state_reference_horizon.clear();
  input_reference_horizon.clear();
  state_reference_horizon.reserve(static_cast<std::size_t>(nmpc_parameters_.horizon_steps + 1));
  input_reference_horizon.reserve(static_cast<std::size_t>(nmpc_parameters_.horizon_steps));

  for (int offset = 0; offset <= nmpc_parameters_.horizon_steps; ++offset) {
    const std::size_t index = std::min(start_index + static_cast<std::size_t>(offset), active_trajectory_.samples.size() - 1);
    const auto & sample = active_trajectory_.samples[index];
    state_reference_horizon.push_back({sample.x, sample.y, sample.psi, sample.vx});
  }

  for (int offset = 0; offset < nmpc_parameters_.horizon_steps; ++offset) {
    const std::size_t index = std::min(start_index + static_cast<std::size_t>(offset), active_trajectory_.samples.size() - 1);
    const auto & sample = active_trajectory_.samples[index];
    input_reference_horizon.push_back({sample.delta, sample.ax});
  }
}

// Publish the cached fixed reference from P0 to PN for RViz.
void QCar2NMPCController::publishReferencePreviewPath()
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

      // Preview is a sampled Bezier path of the fixed reference P0 -> ... -> PN.
      // Because preview_start_pose is exactly P0, generateFullReferenceTrajectory()
      // skips the artificial S0 -> P0 start segment and returns only the fixed reference.
      const Pose2D preview_start_pose{points.front()[0], points.front()[1], trajectory_parameters_.theta_start};
      const auto preview_trajectory = trajectory_generator_->generateFullReferenceTrajectory(preview_start_pose);
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
      RCLCPP_ERROR(this->get_logger(), "Cannot build fixed reference preview path: %s", exception.what());
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

// Publish the active path from the current pose through the fixed reference.
void QCar2NMPCController::publishActivePath()
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

// Republish cached paths so RViz displays can connect after launch.
void QCar2NMPCController::pathPublishTimerCallback()
{
  // Keep the two visualization topics alive for RViz without regenerating Bezier curves.
  if (has_reference_path_msg_ && reference_path_publisher_ && reference_path_publisher_->get_subscription_count() > 0) {
    publishReferencePreviewPath();
  }
  if (has_active_path_msg_ && active_path_publisher_ && active_path_publisher_->get_subscription_count() > 0) {
    publishActivePath();
  }
}

// Publish a zero steering, speed, and acceleration command.
void QCar2NMPCController::publishZeroCommand()
{
  publishMotorCommand(0.0, 0.0, 0.0);
}

// Publish one steering, speed, and acceleration command to the hardware node.
void QCar2NMPCController::publishMotorCommand(
  double steering_angle,
  double longitudinal_velocity_command,
  double longitudinal_acceleration)
{
  qcar2_interfaces::msg::MotorCommands command;

  command.motor_names = {"steering_angle", "desired_speed", "desired_acceleration"};
  command.values = {steering_angle, longitudinal_velocity_command, longitudinal_acceleration};
  command_publisher_->publish(command);
}


// Stop safely, export the run, and clear the stored warm-start trajectory.
void QCar2NMPCController::stopTracking(const std::string & reason)
{
  publishZeroCommand();
  solver_->resetWarmStart();
  if (result_logger_) {
    result_logger_->exportRunResults(reason, this->get_logger());
  }
  mode_ = ControllerMode::WaitingForGoal;
  RCLCPP_INFO(this->get_logger(), "NMPC tracking stopped: %s. Waiting for a new 2D Goal.", reason.c_str());
}

// Convert one quaternion into planar yaw.
double QCar2NMPCController::yawFromQuaternion(double x, double y, double z, double w)
{
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

// Convert one planar yaw into a quaternion for RViz paths.
geometry_msgs::msg::Quaternion QCar2NMPCController::quaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = std::sin(0.5 * yaw);
  quaternion.w = std::cos(0.5 * yaw);
  return quaternion;
}

// Clamp one value to a closed interval.
double QCar2NMPCController::clip(double value, double lower_bound, double upper_bound)
{
  return std::min(std::max(value, lower_bound), upper_bound);
}


}  // namespace qcar2_nmpc

// Start and spin the NMPC controller node.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<qcar2_nmpc::QCar2NMPCController>());
  rclcpp::shutdown();
  return 0;
}
