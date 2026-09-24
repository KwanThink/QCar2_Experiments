#ifndef QCAR2_NMPC_CONTROLLER_HPP_
#define QCAR2_NMPC_CONTROLLER_HPP_

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

namespace qcar2_nmpc
{

struct VehicleState
{
  double x = 0.0;
  double y = 0.0;
  double psi = 0.0;
  double vx = 0.0;
};

struct NMPCParameters
{
  double wheelbase = 0.25725;
  double sample_time = 0.02;
  int horizon_steps = 15;
  double vx_min = -3.0;
  double vx_max = 3.0;
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
  bool success = false;
};


class AcadosSingleTrackSolver
{
public:
  // Store fixed NMPC dimensions and constraints used by the generated solver.
  explicit AcadosSingleTrackSolver(const NMPCParameters & parameters);

  // Release the generated acados solver resources.
  ~AcadosSingleTrackSolver();

  // Initialize the generated acados C solver when it is available at build time.
  // Without generated files this function keeps the node safe and returns false.
  bool initialize(const rclcpp::Logger & logger);

  // Solve one SQP_RTI control step using current state and the reference horizon.
  // The returned input order follows the simulation convention: [delta, ax].
  SolverResult solve(
    const std::array<double, 4> & current_state,
    const std::vector<std::array<double, 4>> & state_reference_horizon,
    const std::vector<std::array<double, 2>> & input_reference_horizon,
    const std::array<double, 2> & input_initial_guess);

  // Clear the stored optimal trajectory before a new tracking phase.
  void resetWarmStart();

  // Return whether the generated acados solver is ready to use.
  bool isAvailable() const { return solver_available_; }

private:
  NMPCParameters parameters_;
  bool solver_available_ = false;
  bool has_warm_start_ = false;
  std::vector<std::array<double, 4>> previous_state_solution_;
  std::vector<std::array<double, 2>> previous_input_solution_;

#ifdef QCAR2_HAS_ACADOS_SOLVER
  void * solver_capsule_ = nullptr;
#endif
};

class QCar2NMPCController : public rclcpp::Node
{
public:
  // Create ROS interfaces, load parameters, and initialize the trajectory and solver objects.
  QCar2NMPCController();

private:
  enum class ControllerMode
  {
    WaitingForGoal,
    Tracking
  };

  // Read all ROS parameters used by the controller, model, constraints, and trajectory generator.
  // Defaults are chosen to match the single-track simulation unless overridden by YAML.
  void declareAndLoadParameters();

  // Receive measured wheel/motor speed and convert it to longitudinal velocity vx in m/s.
  // This conversion mirrors qcar2_hardware and qcar2_odometry.
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // Process a new RViz2 2D Goal. If the car is moving, stop first, discard old reference,
  // generate a new Bezier reference from the current pose, and then arm tracking.
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // Main periodic control loop: read TF state, check safety, solve NMPC, integrate ax to vx_cmd,
  // and publish steering_angle/motor_throttle to qcar2_hardware.
  void controlTimerCallback();

  // Read map -> base_link and the latest velocity feedback to form state [X,Y,psi,vx].
  // Returns false if TF or velocity data are missing/stale.
  bool readCurrentVehicleState(VehicleState & state);

  // Build the NMPC state/input reference horizon from the generated trajectory.
  // The index is clipped near the end so the terminal reference remains valid.
  void buildReferenceHorizon(
    std::size_t start_index,
    std::vector<std::array<double, 4>> & state_reference_horizon,
    std::vector<std::array<double, 2>> & input_reference_horizon) const;

  // Publish the current generated reference trajectory as nav_msgs/Path for RViz2.
  // This lets the user inspect the path before and during tracking.
  void publishReferencePath();

  // Publish a zero command immediately. This is called for stop, solver failure,
  // missing TF, missing velocity feedback, and inactive waiting state.
  void publishZeroCommand();

  // Publish one command to qcar2_hardware with steering_angle = delta and motor_throttle = vx_cmd.
  // motor_throttle is a velocity command because qcar2_hardware closes the speed loop internally.
  void publishMotorCommand(
    double steering_angle,
    double longitudinal_velocity_command,
    double longitudinal_acceleration);
  // Stop tracking safely and return to the waiting state.
  // The controller then waits for the next RViz2 goal.
  void stopTracking(const std::string & reason);


  // Validate a received goal pose before trajectory generation.
  // A goal must have finite position and finite yaw.
  bool isGoalPoseValid(const geometry_msgs::msg::PoseStamped & goal_pose) const;

  // Convert a quaternion message to planar yaw.
  // The formula is used to avoid adding unnecessary dependencies.
  static double yawFromQuaternion(
    double x,
    double y,
    double z,
    double w);

  // Convert a planar yaw angle to a geometry_msgs quaternion.
  // The reference path uses this for RViz2 visualization.
  static geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw);

  // Clip a value into a closed interval.
  // Used for commands and integrated longitudinal speed.
  static double clip(double value, double lower_bound, double upper_bound);

  std::string map_frame_ = "map";
  std::string base_frame_ = "base_link";
  std::string goal_topic_ = "/goal_pose";
  std::string joint_state_topic_ = "qcar2_joint";
  std::string command_topic_ = "qcar2_motor_speed_cmd";
  std::string reference_path_topic_ = "/qcar2_nmpc/reference_path";
  std::string result_directory_ = "/home/nvidia/Results/nmpc_Mk1";
  std::string source_config_file_;

  double control_period_ = 0.02;
  double transform_timeout_ = 0.1;
  double feedback_timeout_ = 0.25;
  double goal_position_tolerance_ = 0.08;
  double goal_yaw_tolerance_ = 0.10;
  bool stop_on_solver_failure_ = true;
  bool enable_result_logging_ = true;

  NMPCParameters nmpc_parameters_;
  TrajectoryParameters trajectory_parameters_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Publisher<qcar2_interfaces::msg::MotorCommands>::SharedPtr command_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::unique_ptr<QCar2BezierTrajectoryGenerator> trajectory_generator_;
  std::unique_ptr<AcadosSingleTrackSolver> solver_;

  ControllerMode mode_ = ControllerMode::WaitingForGoal;
  ReferenceTrajectory active_trajectory_;
  Pose2D active_goal_pose_;
  rclcpp::Time tracking_start_time_;
  double velocity_feedback_ = 0.0;
  double longitudinal_velocity_command_ = 0.0;
  rclcpp::Time last_velocity_feedback_time_;
  bool has_velocity_feedback_ = false;
  std::array<double, 2> last_control_input_{{0.0, 0.0}};
  std::unique_ptr<ResultLogger> result_logger_;
};

}  // namespace qcar2_nmpc

#endif  // QCAR2_NMPC_CONTROLLER_HPP_
