#ifndef QCAR2_FLMPC_CONTROLLER_HPP_
#define QCAR2_FLMPC_CONTROLLER_HPP_

#include "qcar2_traj.hpp"
#include "result_logger.hpp"

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "qcar2_interfaces/msg/motor_commands.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace qcar2_flmpc
{

struct VehicleState
{
  double x = 0.0;
  double y = 0.0;
  double psi = 0.0;
  double vx = 0.0;
};

struct FLMPCParameters
{
  double wheelbase = 0.25725;
  double epsilon = 0.001;
  double sample_time = 0.03;
  int horizon_steps = 30;
  double vx_min = -2.0;
  double vx_max = 2.0;
  double delta_min = -0.5236;
  double delta_max = 0.5236;
  double ax_min = -1.0;
  double ax_max = 1.0;
};

struct SolverResult
{
  int status = -1;
  double delta = 0.0;
  double ax = 0.0;
  double v1 = 0.0;
  double v2 = 0.0;
  bool success = false;
};

class AcadosFLMPCSolver
{
public:
  // Store the fixed FLMPC dimensions and physical limits.
  explicit AcadosFLMPCSolver(const FLMPCParameters & parameters);

  // Release the generated acados solver resources.
  ~AcadosFLMPCSolver();

  // Initialize the generated FLMPC acados solver.
  bool initialize(const rclcpp::Logger & logger);

  // Solve one FLMPC step using measured state and reference horizons.
  SolverResult solve(
    const std::array<double, 4> & current_physical_state,
    const std::vector<std::array<double, 4>> & state_reference_horizon,
    const std::vector<std::array<double, 2>> & input_reference_horizon);

  // Clear the stored optimal flat-state and virtual-input sequences.
  void resetWarmStart();

  // Return whether the generated acados solver is ready.
  bool isAvailable() const { return solver_available_; }

private:
  FLMPCParameters parameters_;
  bool solver_available_ = false;
  bool has_warm_start_ = false;
  std::vector<std::array<double, 4>> previous_flat_state_solution_;
  std::vector<std::array<double, 2>> previous_virtual_input_solution_;

#ifdef QCAR2_HAS_ACADOS_SOLVER
  void * solver_capsule_ = nullptr;
#endif
};

class QCar2FLMPCController : public rclcpp::Node
{
public:
  // Create ROS interfaces and initialize FLMPC controller components.
  QCar2FLMPCController();

private:
  enum class ControllerMode
  {
    WaitingForGoal,
    Tracking
  };

  // Read all controller, solver, and trajectory parameters.
  void declareAndLoadParameters();

  // Convert motor-speed feedback into longitudinal velocity.
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // Generate a new online reference from an RViz2 goal.
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // Execute one closed-loop FLMPC tracking step.
  void controlTimerCallback();

  // Read the latest pose and signed longitudinal velocity.
  bool readCurrentVehicleState(VehicleState & state);

  // Build physical reference horizons for the FLMPC solver.
  void buildReferenceHorizon(
    std::size_t start_index,
    std::vector<std::array<double, 4>> & state_reference_horizon,
    std::vector<std::array<double, 2>> & input_reference_horizon) const;

  // Publish the active reference path for RViz2.
  void publishReferencePath();

  // Publish an immediate zero command.
  void publishZeroCommand();

  // Publish steering, desired speed, and acceleration to hardware.
  void publishMotorCommand(
    double steering_angle,
    double longitudinal_velocity_command,
    double longitudinal_acceleration);

  // Stop tracking and export the completed run.
  void stopTracking(const std::string & reason);

  // Validate a received goal pose before trajectory generation.
  bool isGoalPoseValid(const geometry_msgs::msg::PoseStamped & goal_pose) const;

  // Convert a quaternion message to planar yaw.
  static double yawFromQuaternion(
    double x,
    double y,
    double z,
    double w);

  // Convert a planar yaw angle to a quaternion.
  static geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw);

  // Clip a scalar into a closed interval.
  static double clip(double value, double lower_bound, double upper_bound);

  std::string map_frame_ = "map";
  std::string base_frame_ = "base_link";
  std::string goal_topic_ = "/goal_pose";
  std::string joint_state_topic_ = "qcar2_joint";
  std::string command_topic_ = "qcar2_motor_speed_cmd";
  std::string reference_path_topic_ = "/qcar2_flmpc/reference_path";
  std::string result_directory_ = "/home/nvidia/Results/flmpc_Mk1";
  std::string source_config_file_;

  double control_period_ = 0.03;
  double transform_timeout_ = 0.1;
  double feedback_timeout_ = 0.25;
  double goal_position_tolerance_ = 0.10;
  double launch_velocity_threshold_ = 0.05;
  bool stop_on_solver_failure_ = true;
  bool enable_result_logging_ = true;

  FLMPCParameters flmpc_parameters_;
  TrajectoryParameters trajectory_parameters_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Publisher<qcar2_interfaces::msg::MotorCommands>::SharedPtr command_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::unique_ptr<QCar2BezierTrajectoryGenerator> trajectory_generator_;
  std::unique_ptr<AcadosFLMPCSolver> solver_;

  ControllerMode mode_ = ControllerMode::WaitingForGoal;
  ReferenceTrajectory active_trajectory_;
  Pose2D active_goal_pose_;
  rclcpp::Time tracking_start_time_;
  rclcpp::Time reference_start_time_;
  double velocity_feedback_ = 0.0;
  double longitudinal_velocity_command_ = 0.0;
  rclcpp::Time last_velocity_feedback_time_;
  bool has_velocity_feedback_ = false;
  bool reference_clock_started_ = false;
  std::unique_ptr<ResultLogger> result_logger_;
};

}  // namespace qcar2_flmpc

#endif  // QCAR2_FLMPC_CONTROLLER_HPP_
