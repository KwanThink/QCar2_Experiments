#include "qcar2_flmpc_controller.hpp"

#include <algorithm>
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
#include "acados_solver_qcar2_flmpc.h"
#include "acados_c/ocp_nlp_interface.h"
}
#endif

namespace qcar2_flmpc
{
namespace
{
constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kNominalSpeedThreshold = 1.0e-4;
constexpr double kRawMotorSpeedToLinearVelocity =  (1.0 / (720.0 * 4.0)) * ((13.0 * 19.0) / (70.0 * 37.0)) * (2.0 * kPi) * 0.033;

// Return true when a floating-point value is finite.
bool isFinite(double value)
{
  return std::isfinite(value);
}

// Return the Euclidean distance from the vehicle to the goal.
double distanceToGoal(const VehicleState & state, const Pose2D & goal_pose)
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

// Transform physical state [X,Y,psi,vx] into flat state [X,Xdot,Y,Ydot].
std::array<double, 4> physicalStateToFlatState(const std::array<double, 4> & physical_state)
{
  const double heading = physical_state[2];
  const double signed_velocity = physical_state[3];
  return {
    physical_state[0],
    signed_velocity * std::cos(heading),
    physical_state[1],
    signed_velocity * std::sin(heading)};
}

// Transform reference physical inputs into a virtual-input initial guess.
std::array<double, 2> physicalInputToVirtualInput(
  const std::array<double, 4> & physical_state,
  const std::array<double, 2> & physical_input,
  double wheelbase)
{
  const double heading = physical_state[2];
  const double signed_velocity = physical_state[3];
  const double steering_angle = physical_input[0];
  const double longitudinal_acceleration = physical_input[1];
  const double lateral_acceleration = (signed_velocity * signed_velocity / wheelbase) * std::tan(steering_angle);

  const double v1 =  longitudinal_acceleration * std::cos(heading) - lateral_acceleration * std::sin(heading);
  const double v2 =  longitudinal_acceleration * std::sin(heading) + lateral_acceleration * std::cos(heading);
  return {v1, v2};
}

// Reconstruct nominal heading and signed speed from one flat prediction stage.
std::array<double, 2> reconstructNominalPhysicalState(
  const std::array<double, 4> & flat_state,
  const std::array<double, 4> & reference_physical_state)
{
  const double x_velocity = flat_state[1];
  const double y_velocity = flat_state[3];
  const double flat_speed = std::hypot(x_velocity, y_velocity);
  const double reference_heading = reference_physical_state[2];
  const double reference_signed_velocity = reference_physical_state[3];

  if (flat_speed <= kNominalSpeedThreshold) {
    return {reference_heading, reference_signed_velocity};
  }

  double direction_sign = 1.0;
  if (reference_signed_velocity < -kNominalSpeedThreshold) {
    direction_sign = -1.0;
  } else if (reference_signed_velocity <= kNominalSpeedThreshold) {
    const double velocity_along_reference_heading = x_velocity * std::cos(reference_heading) + y_velocity * std::sin(reference_heading);
    direction_sign = velocity_along_reference_heading < 0.0 ? -1.0 : 1.0;
  }

  double body_heading = std::atan2(y_velocity, x_velocity);
  if (direction_sign < 0.0) {
    body_heading += kPi;
  }
  body_heading = unwrapToReference(body_heading, reference_heading);

  return {body_heading, direction_sign * flat_speed};
}

// Build one column-major D matrix for the mapped physical-input constraints.
std::array<double, 4> buildVirtualInputConstraintMatrix(
  double heading,
  double signed_velocity,
  double wheelbase,
  double epsilon)
{
  const double denominator = signed_velocity * signed_velocity + epsilon;
  const double steering_scale = wheelbase / denominator;

  const double m00 = -steering_scale * std::sin(heading);
  const double m01 = steering_scale * std::cos(heading);
  const double m10 = std::cos(heading);
  const double m11 = std::sin(heading);

  return {m00, m10, m01, m11};
}

}  // namespace

// Store the FLMPC dimensions and physical limits.
AcadosFLMPCSolver::AcadosFLMPCSolver(const FLMPCParameters & parameters)
: parameters_(parameters)
{
}

// Release the generated acados solver resources.
AcadosFLMPCSolver::~AcadosFLMPCSolver()
{
#ifdef QCAR2_HAS_ACADOS_SOLVER
  if (solver_capsule_ != nullptr) {
    auto * typed_capsule = static_cast<qcar2_flmpc_solver_capsule *>(solver_capsule_);
    qcar2_flmpc_acados_free(typed_capsule);
    qcar2_flmpc_acados_free_capsule(typed_capsule);
    solver_capsule_ = nullptr;
  }
#endif
}

// Create and initialize the generated FLMPC solver capsule.
bool AcadosFLMPCSolver::initialize(const rclcpp::Logger & logger)
{
#ifdef QCAR2_HAS_ACADOS_SOLVER
  auto * typed_capsule = qcar2_flmpc_acados_create_capsule();
  if (typed_capsule == nullptr) {
    RCLCPP_ERROR(logger, "Failed to allocate qcar2_flmpc acados capsule.");
    solver_available_ = false;
    return false;
  }

  const int status = qcar2_flmpc_acados_create(typed_capsule);
  if (status != 0) {
    RCLCPP_ERROR(logger, "qcar2_flmpc_acados_create failed with status %d.", status);
    qcar2_flmpc_acados_free_capsule(typed_capsule);
    solver_available_ = false;
    return false;
  }

  solver_capsule_ = typed_capsule;
  solver_available_ = true;
  RCLCPP_INFO(logger, "Generated acados solver is available for qcar2_flmpc_controller.");
  return true;
#else
  (void)logger;
  solver_available_ = false;
  return false;
#endif
}

// Solve one FLMPC step with shifted warm start and stage-dependent D matrices.
SolverResult AcadosFLMPCSolver::solve(
  const std::array<double, 4> & current_physical_state,
  const std::vector<std::array<double, 4>> & state_reference_horizon,
  const std::vector<std::array<double, 2>> & input_reference_horizon)
{
  SolverResult result;

#ifndef QCAR2_HAS_ACADOS_SOLVER
  (void)current_physical_state;
  (void)state_reference_horizon;
  (void)input_reference_horizon;
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

  auto * capsule = static_cast<qcar2_flmpc_solver_capsule *>(solver_capsule_);
  ocp_nlp_config * nlp_config = qcar2_flmpc_acados_get_nlp_config(capsule);
  ocp_nlp_dims * nlp_dims = qcar2_flmpc_acados_get_nlp_dims(capsule);
  ocp_nlp_in * nlp_in = qcar2_flmpc_acados_get_nlp_in(capsule);
  ocp_nlp_out * nlp_out = qcar2_flmpc_acados_get_nlp_out(capsule);

  const std::array<double, 4> current_flat_state = physicalStateToFlatState(current_physical_state);
  double x0[4] = {
    current_flat_state[0],
    current_flat_state[1],
    current_flat_state[2],
    current_flat_state[3]};
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", x0);
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", x0);

  const bool use_warm_start =
    has_warm_start_ &&
    previous_flat_state_solution_.size() == horizon_steps + 1U &&
    previous_virtual_input_solution_.size() == horizon_steps;

  std::vector<std::array<double, 4>> flat_state_guesses(horizon_steps + 1U);
  std::vector<std::array<double, 2>> virtual_input_guesses(horizon_steps);

  for (std::size_t stage = 0; stage <= horizon_steps; ++stage) {
    if (use_warm_start) {
      const std::size_t shifted_state_index = std::min(stage + 1U, horizon_steps);
      flat_state_guesses[stage] = previous_flat_state_solution_[shifted_state_index];
    } else {
      flat_state_guesses[stage] = physicalStateToFlatState(state_reference_horizon[stage]);
    }
  }
  flat_state_guesses[0] = current_flat_state;

  for (std::size_t stage = 0; stage < horizon_steps; ++stage) {
    if (use_warm_start) {
      const std::size_t shifted_input_index = std::min(stage + 1U, horizon_steps - 1U);
      virtual_input_guesses[stage] = previous_virtual_input_solution_[shifted_input_index];
    } else {
      virtual_input_guesses[stage] = physicalInputToVirtualInput(
        state_reference_horizon[stage],
        input_reference_horizon[stage],
        parameters_.wheelbase);
    }
  }

  for (int stage = 0; stage < parameters_.horizon_steps; ++stage) {
    const std::size_t stage_index = static_cast<std::size_t>(stage);

    double yref[4] = {
      state_reference_horizon[stage_index + 1U][0],
      state_reference_horizon[stage_index + 1U][1],
      0.0,
      0.0};
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, stage, "yref", yref);

    double nominal_heading = current_physical_state[2];
    double nominal_signed_velocity = current_physical_state[3];
    if (stage > 0) {
      const std::array<double, 2> nominal_physical_state = reconstructNominalPhysicalState(
        flat_state_guesses[stage_index],
        state_reference_horizon[stage_index]);
      nominal_heading = nominal_physical_state[0];
      nominal_signed_velocity = nominal_physical_state[1];
    }

    const std::array<double, 4> constraint_matrix = buildVirtualInputConstraintMatrix(
      nominal_heading,
      nominal_signed_velocity,
      parameters_.wheelbase,
      parameters_.epsilon);
    double D[4] = {
      constraint_matrix[0],
      constraint_matrix[1],
      constraint_matrix[2],
      constraint_matrix[3]};
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, stage, "D", D);

    const auto & state_guess = flat_state_guesses[stage_index];
    const auto & input_guess = virtual_input_guesses[stage_index];
    double x_guess[4] = {
      state_guess[0],
      state_guess[1],
      state_guess[2],
      state_guess[3]};
    double u_guess[2] = {input_guess[0], input_guess[1]};
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, stage, "x", x_guess);
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, stage, "u", u_guess);
  }

  const auto & terminal_state_guess = flat_state_guesses.back();
  double x_terminal_guess[4] = {
    terminal_state_guess[0],
    terminal_state_guess[1],
    terminal_state_guess[2],
    terminal_state_guess[3]};
  ocp_nlp_out_set(
    nlp_config,
    nlp_dims,
    nlp_out,
    nlp_in,
    parameters_.horizon_steps,
    "x",
    x_terminal_guess);

  result.status = qcar2_flmpc_acados_solve(capsule);
  if (result.status != 0) {
    resetWarmStart();
    result.success = false;
    return result;
  }

  double optimal_virtual_input[2] = {0.0, 0.0};
  ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", optimal_virtual_input);
  if (!isFinite(optimal_virtual_input[0]) || !isFinite(optimal_virtual_input[1])) {
    resetWarmStart();
    result.status = -96;
    result.success = false;
    return result;
  }

  std::vector<std::array<double, 4>> flat_state_solution(horizon_steps + 1U);
  std::vector<std::array<double, 2>> virtual_input_solution(horizon_steps);
  bool solution_is_finite = true;

  for (int stage = 0; stage <= parameters_.horizon_steps; ++stage) {
    double optimal_flat_state[4] = {0.0, 0.0, 0.0, 0.0};
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, stage, "x", optimal_flat_state);
    flat_state_solution[stage] = {
      optimal_flat_state[0],
      optimal_flat_state[1],
      optimal_flat_state[2],
      optimal_flat_state[3]};
    solution_is_finite =
      solution_is_finite &&
      isFinite(optimal_flat_state[0]) &&
      isFinite(optimal_flat_state[1]) &&
      isFinite(optimal_flat_state[2]) &&
      isFinite(optimal_flat_state[3]);
  }

  for (int stage = 0; stage < parameters_.horizon_steps; ++stage) {
    double optimal_stage_virtual_input[2] = {0.0, 0.0};
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, stage, "u", optimal_stage_virtual_input);
    virtual_input_solution[stage] = {
      optimal_stage_virtual_input[0],
      optimal_stage_virtual_input[1]};
    solution_is_finite =
      solution_is_finite &&
      isFinite(optimal_stage_virtual_input[0]) &&
      isFinite(optimal_stage_virtual_input[1]);
  }

  if (solution_is_finite) {
    previous_flat_state_solution_ = std::move(flat_state_solution);
    previous_virtual_input_solution_ = std::move(virtual_input_solution);
    has_warm_start_ = true;
  } else {
    resetWarmStart();
  }

  result.v1 = optimal_virtual_input[0];
  result.v2 = optimal_virtual_input[1];

  const double measured_heading = current_physical_state[2];
  const double measured_signed_velocity = current_physical_state[3];
  const double denominator =
    measured_signed_velocity * measured_signed_velocity + parameters_.epsilon;
  result.delta = std::atan(
    (parameters_.wheelbase / denominator) *
    (-result.v1 * std::sin(measured_heading) + result.v2 * std::cos(measured_heading)));
  result.ax =
    result.v1 * std::cos(measured_heading) + result.v2 * std::sin(measured_heading);

  if (!isFinite(result.delta) || !isFinite(result.ax)) {
    resetWarmStart();
    result.status = -95;
    result.success = false;
    return result;
  }

  result.success = true;
  return result;
#endif
}

// Clear stored predictions used by warm start and nominal constraints.
void AcadosFLMPCSolver::resetWarmStart()
{
  previous_flat_state_solution_.clear();
  previous_virtual_input_solution_.clear();
  has_warm_start_ = false;
}

// Create ROS interfaces and initialize the FLMPC controller.
QCar2FLMPCController::QCar2FLMPCController(): Node("qcar2_flmpc_controller")
{
  declareAndLoadParameters();
  result_logger_ = std::make_unique<ResultLogger>(
    enable_result_logging_,
    result_directory_,
    "run_FLMPC_Mk1_",
    source_config_file_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  trajectory_generator_ = std::make_unique<QCar2BezierTrajectoryGenerator>(trajectory_parameters_);
  solver_ = std::make_unique<AcadosFLMPCSolver>(flmpc_parameters_);
  if (!solver_->initialize(this->get_logger())) {
    RCLCPP_WARN(
      this->get_logger(),
      "Generated acados FLMPC solver was not found at build time. The controller will stay safe with zero commands until the solver is generated and the package is rebuilt.");
  }

  command_publisher_ = this->create_publisher<qcar2_interfaces::msg::MotorCommands>(command_topic_, 10);
  reference_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(reference_path_topic_, 10);

  joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
    joint_state_topic_,
    10,
    std::bind(&QCar2FLMPCController::jointStateCallback, this, std::placeholders::_1));

  goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic_,
    10,
    std::bind(&QCar2FLMPCController::goalCallback, this, std::placeholders::_1));

  const auto period = std::chrono::duration<double>(control_period_);
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&QCar2FLMPCController::controlTimerCallback, this));

  publishZeroCommand();
  RCLCPP_INFO(
    this->get_logger(),
    "qcar2_flmpc_controller is waiting for map -> base_link and /goal_pose.");
}

// Declare and load controller, solver, and trajectory parameters.
void QCar2FLMPCController::declareAndLoadParameters()
{
  this->declare_parameter("map_frame", map_frame_);
  this->declare_parameter("base_frame", base_frame_);
  this->declare_parameter("goal_topic", goal_topic_);
  this->declare_parameter("joint_state_topic", joint_state_topic_);
  this->declare_parameter("command_topic", command_topic_);
  this->declare_parameter("reference_path_topic", reference_path_topic_);
  this->declare_parameter("result_directory", result_directory_);
  this->declare_parameter("source_config_file", source_config_file_);
  this->declare_parameter("enable_result_logging", enable_result_logging_);
  this->declare_parameter("control_period", control_period_);
  this->declare_parameter("transform_timeout", transform_timeout_);
  this->declare_parameter("feedback_timeout", feedback_timeout_);
  this->declare_parameter("goal_position_tolerance", goal_position_tolerance_);
  this->declare_parameter("launch_velocity_threshold", launch_velocity_threshold_);
  this->declare_parameter("stop_on_solver_failure", stop_on_solver_failure_);

  this->declare_parameter("wheelbase", flmpc_parameters_.wheelbase);
  this->declare_parameter("epsilon", flmpc_parameters_.epsilon);
  this->declare_parameter("Ts", flmpc_parameters_.sample_time);
  this->declare_parameter("mpc_N", flmpc_parameters_.horizon_steps);
  this->declare_parameter("vx_min", flmpc_parameters_.vx_min);
  this->declare_parameter("vx_max", flmpc_parameters_.vx_max);
  this->declare_parameter("delta_min", flmpc_parameters_.delta_min);
  this->declare_parameter("delta_max", flmpc_parameters_.delta_max);
  this->declare_parameter("ax_min", flmpc_parameters_.ax_min);
  this->declare_parameter("ax_max", flmpc_parameters_.ax_max);
  this->declare_parameter<std::vector<double>>("Q", {160.0, 160.0});
  this->declare_parameter<std::vector<double>>("R", {2.0, 2.0});

  this->declare_parameter(
    "trajectory_generation_mode",
    trajectory_parameters_.trajectory_generation_mode);
  this->declare_parameter("number_of_waypoints", trajectory_parameters_.number_of_waypoints);
  this->declare_parameter<std::vector<double>>("segment_times", trajectory_parameters_.segment_times);
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
  result_directory_ = this->get_parameter("result_directory").as_string();
  source_config_file_ = this->get_parameter("source_config_file").as_string();
  enable_result_logging_ = this->get_parameter("enable_result_logging").as_bool();
  control_period_ = this->get_parameter("control_period").as_double();
  transform_timeout_ = this->get_parameter("transform_timeout").as_double();
  feedback_timeout_ = this->get_parameter("feedback_timeout").as_double();
  goal_position_tolerance_ = this->get_parameter("goal_position_tolerance").as_double();
  launch_velocity_threshold_ = this->get_parameter("launch_velocity_threshold").as_double();
  stop_on_solver_failure_ = this->get_parameter("stop_on_solver_failure").as_bool();

  flmpc_parameters_.wheelbase = this->get_parameter("wheelbase").as_double();
  flmpc_parameters_.epsilon = this->get_parameter("epsilon").as_double();
  flmpc_parameters_.sample_time = this->get_parameter("Ts").as_double();
  flmpc_parameters_.horizon_steps = this->get_parameter("mpc_N").as_int();
  flmpc_parameters_.vx_min = this->get_parameter("vx_min").as_double();
  flmpc_parameters_.vx_max = this->get_parameter("vx_max").as_double();
  flmpc_parameters_.delta_min = this->get_parameter("delta_min").as_double();
  flmpc_parameters_.delta_max = this->get_parameter("delta_max").as_double();
  flmpc_parameters_.ax_min = this->get_parameter("ax_min").as_double();
  flmpc_parameters_.ax_max = this->get_parameter("ax_max").as_double();

  trajectory_parameters_.wheelbase = flmpc_parameters_.wheelbase;
  trajectory_parameters_.sample_time = flmpc_parameters_.sample_time;
  trajectory_parameters_.trajectory_generation_mode =
    this->get_parameter("trajectory_generation_mode").as_string();
  trajectory_parameters_.number_of_waypoints =
    this->get_parameter("number_of_waypoints").as_int();
  trajectory_parameters_.segment_times = this->get_parameter("segment_times").as_double_array();
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
void QCar2FLMPCController::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
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

// Stop the current run and generate a new online reference.
void QCar2FLMPCController::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  publishZeroCommand();
  mode_ = ControllerMode::WaitingForGoal;
  reference_clock_started_ = false;
  solver_->resetWarmStart();

  if (!isGoalPoseValid(*msg)) {
    RCLCPP_WARN(this->get_logger(), "Received invalid /goal_pose. The car will remain stopped.");
    return;
  }

  VehicleState current_state;
  if (!readCurrentVehicleState(current_state)) {
    RCLCPP_WARN(
      this->get_logger(),
      "Cannot create trajectory because current state is unavailable.");
    publishZeroCommand();
    return;
  }

  const Pose2D start_pose{current_state.x, current_state.y, current_state.psi};
  const Pose2D goal_pose{msg->pose.position.x, msg->pose.position.y, yawFromQuaternion(
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w)};

  try {
    active_trajectory_ = trajectory_generator_->generateTrajectory(start_pose, goal_pose);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(this->get_logger(), "Trajectory generation failed: %s", exception.what());
    publishZeroCommand();
    return;
  }

  active_goal_pose_ = goal_pose;
  longitudinal_velocity_command_ = clip(current_state.vx, flmpc_parameters_.vx_min, flmpc_parameters_.vx_max);

  if (result_logger_) {
    result_logger_->reset();
  }
  tracking_start_time_ = this->get_clock()->now();
  reference_start_time_ = tracking_start_time_;
  reference_clock_started_ = std::abs(current_state.vx) >= launch_velocity_threshold_;
  mode_ = ControllerMode::Tracking;

  publishReferencePath();
  RCLCPP_INFO(this->get_logger(), "New FLMPC reference is ready: %zu samples, %.3f s duration. Launch synchronization threshold is |vx| >= %.3f m/s.", active_trajectory_.samples.size(), active_trajectory_.duration(), launch_velocity_threshold_);
}

// Execute one closed-loop FLMPC tracking step.
void QCar2FLMPCController::controlTimerCallback()
{
  if (mode_ != ControllerMode::Tracking) {
    publishZeroCommand();
    return;
  }

  VehicleState current_state;
  if (!readCurrentVehicleState(current_state)) {
    publishZeroCommand();
    return;
  }

  if (active_trajectory_.empty()) {
    stopTracking("active reference trajectory is empty");
    return;
  }

  const rclcpp::Time current_time = this->get_clock()->now();
  const double elapsed_time = (current_time - tracking_start_time_).seconds();
  if (!reference_clock_started_ && std::abs(current_state.vx) >= launch_velocity_threshold_) {
    reference_start_time_ = current_time;
    reference_clock_started_ = true;
    RCLCPP_INFO(this->get_logger(), "Launch synchronization completed at |vx| = %.3f m/s. The reference clock starts now.", std::abs(current_state.vx));
  }
  const double reference_elapsed_time = reference_clock_started_ ? (current_time - reference_start_time_).seconds() : 0.0;
  const std::size_t last_reference_index = active_trajectory_.samples.size() - 1U;
  const std::size_t elapsed_reference_index = static_cast<std::size_t>(std::max(0.0, std::round(reference_elapsed_time / flmpc_parameters_.sample_time)));
  const std::size_t reference_index = std::min(elapsed_reference_index, last_reference_index);

  const bool reference_finished = reference_clock_started_ && reference_elapsed_time >= active_trajectory_.duration();
  const double goal_distance = distanceToGoal(current_state, active_goal_pose_);
  if (reference_finished && goal_distance <= goal_position_tolerance_)
  {
    stopTracking("final goal position tolerance reached");
    return;
  }

  if (!solver_->isAvailable()) {
    publishZeroCommand();
    return;
  }

  std::vector<std::array<double, 4>> state_reference_horizon;
  std::vector<std::array<double, 2>> input_reference_horizon;
  buildReferenceHorizon(reference_index, state_reference_horizon, input_reference_horizon);

  const double current_psi_for_solver = unwrapToReference(current_state.psi, state_reference_horizon.front()[2]);
  const std::array<double, 4> current_physical_state{{current_state.x, current_state.y, current_psi_for_solver, current_state.vx}};

  const auto solve_start_time = std::chrono::steady_clock::now();
  const SolverResult solver_result = solver_->solve(current_physical_state, state_reference_horizon, input_reference_horizon);
  const auto solve_end_time = std::chrono::steady_clock::now();
  const double solve_time = std::chrono::duration<double>(solve_end_time - solve_start_time).count();

  const auto & log_reference_sample = active_trajectory_.samples[reference_index];

  if (!solver_result.success) {
    if (result_logger_) {
      result_logger_->incrementSolverFailures();
      result_logger_->recordSample(elapsed_time, current_state, log_reference_sample, solver_result, 0.0, solve_time);
    }
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "acados FLMPC solver failed with status %d. Publishing zero command.",
      solver_result.status);
    publishZeroCommand();
    if (stop_on_solver_failure_) {
      stopTracking("Solver failure !!!");
    }
    return;
  }

  const double steering_angle = clip(solver_result.delta, flmpc_parameters_.delta_min, flmpc_parameters_.delta_max);
  const double longitudinal_acceleration = clip(solver_result.ax, flmpc_parameters_.ax_min, flmpc_parameters_.ax_max);

  longitudinal_velocity_command_ = clip(longitudinal_velocity_command_ + longitudinal_acceleration * flmpc_parameters_.sample_time,
    flmpc_parameters_.vx_min, flmpc_parameters_.vx_max);
  const double longitudinal_velocity_command = longitudinal_velocity_command_;

  publishMotorCommand(steering_angle, longitudinal_velocity_command, longitudinal_acceleration);

  SolverResult clipped_solver_result = solver_result;
  clipped_solver_result.delta = steering_angle;
  clipped_solver_result.ax = longitudinal_acceleration;
  if (result_logger_) {
    result_logger_->recordSample(elapsed_time, current_state, log_reference_sample, clipped_solver_result, longitudinal_velocity_command, solve_time);
  }
}

// Read the latest pose and signed longitudinal velocity feedback.
bool QCar2FLMPCController::readCurrentVehicleState(VehicleState & state)
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    (void)transform_timeout_;
    transform = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & exception) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for transform %s -> %s: %s", map_frame_.c_str(), base_frame_.c_str(), exception.what());
    return false;
  }

  if (!has_velocity_feedback_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for qcar2_joint velocity feedback.");
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

  state.x = transform.transform.translation.x;
  state.y = transform.transform.translation.y;
  state.psi = yawFromQuaternion(transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w);
  state.vx = velocity_feedback_;

  return isFinite(state.x) && isFinite(state.y) && isFinite(state.psi) && isFinite(state.vx);
}

// Build a reference horizon and repeat the final sample when needed.
void QCar2FLMPCController::buildReferenceHorizon(
  std::size_t start_index,
  std::vector<std::array<double, 4>> & state_reference_horizon,
  std::vector<std::array<double, 2>> & input_reference_horizon) const
{
  state_reference_horizon.clear();
  input_reference_horizon.clear();
  state_reference_horizon.reserve(static_cast<std::size_t>(flmpc_parameters_.horizon_steps + 1));
  input_reference_horizon.reserve(static_cast<std::size_t>(flmpc_parameters_.horizon_steps));

  for (int offset = 0; offset <= flmpc_parameters_.horizon_steps; ++offset) {
    const std::size_t index = std::min(start_index + static_cast<std::size_t>(offset), active_trajectory_.samples.size() - 1U);
    const auto & sample = active_trajectory_.samples[index];
    state_reference_horizon.push_back({sample.x, sample.y, sample.psi, sample.vx});
  }

  for (int offset = 0; offset < flmpc_parameters_.horizon_steps; ++offset) {
    const std::size_t index = std::min(start_index + static_cast<std::size_t>(offset), active_trajectory_.samples.size() - 1U);
    const auto & sample = active_trajectory_.samples[index];
    input_reference_horizon.push_back({sample.delta, sample.ax});
  }
}

// Publish the active reference path for RViz visualization.
void QCar2FLMPCController::publishReferencePath()
{
  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = this->get_clock()->now();
  path_msg.header.frame_id = map_frame_;

  path_msg.poses.reserve(active_trajectory_.samples.size());
  for (const auto & sample : active_trajectory_.samples) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = sample.x;
    pose.pose.position.y = sample.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = quaternionFromYaw(sample.psi);
    path_msg.poses.push_back(pose);
  }

  reference_path_publisher_->publish(path_msg);
}

// Send zero steering, speed, and acceleration commands.
void QCar2FLMPCController::publishZeroCommand()
{
  publishMotorCommand(0.0, 0.0, 0.0);
}

// Publish one command through the unchanged hardware interface.
void QCar2FLMPCController::publishMotorCommand(
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

// Stop tracking, export results, and clear warm start.
void QCar2FLMPCController::stopTracking(const std::string & reason)
{
  publishZeroCommand();
  solver_->resetWarmStart();
  if (result_logger_) {
    result_logger_->exportRunResults(reason, this->get_logger());
  }
  mode_ = ControllerMode::WaitingForGoal;
  RCLCPP_INFO(
    this->get_logger(),
    "FLMPC tracking stopped: %s. Waiting for a new 2D Goal.",
    reason.c_str());
}

// Validate that the received goal contains finite pose values.
bool QCar2FLMPCController::isGoalPoseValid(
  const geometry_msgs::msg::PoseStamped & goal_pose) const
{
  const double yaw = yawFromQuaternion(
    goal_pose.pose.orientation.x,
    goal_pose.pose.orientation.y,
    goal_pose.pose.orientation.z,
    goal_pose.pose.orientation.w);

  return isFinite(goal_pose.pose.position.x) &&
         isFinite(goal_pose.pose.position.y) &&
         isFinite(goal_pose.pose.orientation.x) &&
         isFinite(goal_pose.pose.orientation.y) &&
         isFinite(goal_pose.pose.orientation.z) &&
         isFinite(goal_pose.pose.orientation.w) &&
         isFinite(yaw);
}

// Convert a quaternion into planar yaw.
double QCar2FLMPCController::yawFromQuaternion(double x, double y, double z, double w)
{
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

// Convert planar yaw into a quaternion.
geometry_msgs::msg::Quaternion QCar2FLMPCController::quaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = std::sin(0.5 * yaw);
  quaternion.w = std::cos(0.5 * yaw);
  return quaternion;
}

// Clip one scalar to the provided closed interval.
double QCar2FLMPCController::clip(double value, double lower_bound, double upper_bound)
{
  return std::min(std::max(value, lower_bound), upper_bound);
}

}  // namespace qcar2_flmpc

// Start the ROS2 FLMPC node and spin until shutdown.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<qcar2_flmpc::QCar2FLMPCController>());
  rclcpp::shutdown();
  return 0;
}
