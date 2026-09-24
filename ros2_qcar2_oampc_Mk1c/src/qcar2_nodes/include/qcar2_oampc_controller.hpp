#ifndef QCAR2_OAMPC_CONTROLLER_HPP_
#define QCAR2_OAMPC_CONTROLLER_HPP_

#include "obstacle_activation.hpp"
#include "obstacle_map.hpp"
#include "oampc_solver.hpp"
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

namespace qcar2_oampc
{

struct VehicleState
{
  double x = 0.0;
  double y = 0.0;
  double psi = 0.0;
  double vx = 0.0;
};

class QCar2OAMPCController : public rclcpp::Node
{
public:
  // Create ROS interfaces and initialize all OAMPC components.
  QCar2OAMPCController();

private:
  enum class ControllerMode
  {
    WaitingForGoal,
    Tracking
  };

  // Read all controller, solver, obstacle, and trajectory parameters.
  void declareAndLoadParameters();

  // Convert motor-speed feedback into longitudinal velocity.
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // Use an RViz2 2D Goal only as a trigger for the predefined trajectory.
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // Execute one closed-loop OAMPC tracking and avoidance step.
  void controlTimerCallback();

  // Read only the current map to base_link pose from TF.
  bool readCurrentPoseFromTF(qcar2_flmpc::Pose2D & pose);

  // Read the latest pose and signed longitudinal velocity.
  bool readCurrentVehicleState(VehicleState & state);

  // Project the current vehicle position onto the active reference path.
  std::size_t projectPositionToReferencePath(double x, double y) const;

  // Build physical reference horizons for the OAMPC solver.
  void buildReferenceHorizon(
    std::size_t start_index,
    std::vector<std::array<double, 4>> & state_reference_horizon,
    std::vector<std::array<double, 2>> & input_reference_horizon) const;

  // Publish the fixed reference path P0 to PN for RViz2 preview.
  void publishReferencePreviewPath();

  // Publish the current active path S0 to P0 to PN.
  void publishActivePath();

  // Republish cached RViz2 paths without regenerating the trajectory.
  void pathPublishTimerCallback();

  // Publish an immediate zero command.
  void publishZeroCommand();

  // Publish steering, desired speed, and acceleration to hardware.
  void publishMotorCommand(
    double steering_angle,
    double longitudinal_velocity_command,
    double longitudinal_acceleration);

  // Stop tracking, export results, and reset online solver state.
  void stopTracking(const std::string & reason);

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
  std::string reference_path_topic_ = "/qcar2_oampc/reference_path";
  std::string active_path_topic_ = "/qcar2_oampc/active_path";
  std::string result_directory_ = "/home/nvidia/Results/oampc_Mk1c";
  std::string source_config_file_;
  std::string obstacle_map_file_ = "/home/nvidia/Maps/obstacle_map_oampc_Mk1c.yaml";

  double control_period_ = 0.03;
  double transform_timeout_ = 0.1;
  double feedback_timeout_ = 0.25;
  double goal_position_tolerance_ = 0.10;
  bool stop_on_solver_failure_ = true;
  bool enable_result_logging_ = true;

  OAMPCParameters oampc_parameters_;
  ObstacleActivationParameters activation_parameters_;
  qcar2_flmpc::TrajectoryParameters trajectory_parameters_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Publisher<qcar2_interfaces::msg::MotorCommands>::SharedPtr command_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr active_path_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr path_publish_timer_;

  double reference_path_z_ = 0.02;
  double active_path_z_ = 0.05;
  nav_msgs::msg::Path reference_path_msg_;
  nav_msgs::msg::Path active_path_msg_;
  bool has_reference_path_msg_ = false;
  bool has_active_path_msg_ = false;

  ObstacleMap obstacle_map_;
  std::unique_ptr<ObstacleActivationManager> obstacle_activation_manager_;
  std::unique_ptr<qcar2_flmpc::QCar2BezierTrajectoryGenerator> trajectory_generator_;
  std::unique_ptr<OAMPCSolver> solver_;

  ControllerMode mode_ = ControllerMode::WaitingForGoal;
  qcar2_flmpc::ReferenceTrajectory active_trajectory_;
  qcar2_flmpc::Pose2D active_goal_pose_;
  rclcpp::Time tracking_start_time_;
  double velocity_feedback_ = 0.0;
  double longitudinal_velocity_command_ = 0.0;
  rclcpp::Time last_velocity_feedback_time_;
  bool has_velocity_feedback_ = false;
  std::size_t reference_progress_index_ = 0U;
  std::unique_ptr<ResultLogger> result_logger_;
};

}  // namespace qcar2_oampc

#endif  // QCAR2_OAMPC_CONTROLLER_HPP_
