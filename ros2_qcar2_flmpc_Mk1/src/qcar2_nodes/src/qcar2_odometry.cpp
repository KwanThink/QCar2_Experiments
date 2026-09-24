#include <cmath>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class QCar2Odometry : public rclcpp::Node
{
public:
  QCar2Odometry()
  : Node("qcar2_odometry")
  {
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "qcar2_joint", 10,
      std::bind(&QCar2Odometry::joint_callback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "qcar2_imu", 50,
      std::bind(&QCar2Odometry::imu_callback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ = this->create_wall_timer(20ms, std::bind(&QCar2Odometry::timer_callback, this));

    last_update_time_ = this->get_clock()->now();

    RCLCPP_INFO(this->get_logger(), "qcar2_odometry started. Publishing /odom and TF odom -> base_link");
  }

private:
  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->velocity.empty()) {
      return;
    }

    // Reuse the same conversion already used in qcar2_hardware.cpp
    // to convert the measured motor/joint speed into linear vehicle speed (m/s).
    linear_velocity_ = (msg->velocity[0] / (720.0 * 4.0)) *
                       ((13.0 * 19.0) / (70.0 * 37.0)) *
                       (2.0 * M_PI) * 0.033;

    has_joint_ = true;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    angular_velocity_z_ = msg->angular_velocity.z;
    has_imu_ = true;
  }

  void timer_callback()
  {
    if (!has_joint_ || !has_imu_) {
      return;
    }

    const rclcpp::Time now = this->get_clock()->now();
    double dt = (now - last_update_time_).seconds();

    if (dt <= 0.0) {
      last_update_time_ = now;
      return;
    }

    if (dt > 0.5) {
      // Ignore an abnormally large dt after startup or pauses.
      dt = 0.02;
    }

    const bool is_stationary = std::abs(linear_velocity_) < stationary_speed_threshold_;

    // Learn the IMU gyro z bias only while the vehicle is stationary.
    // This prevents yaw drift when QCar2 waits for a long time before the next 2D goal.
    if (is_stationary) {
      gyro_bias_z_ = (1.0 - gyro_bias_alpha_) * gyro_bias_z_
                   + gyro_bias_alpha_ * angular_velocity_z_;
    }

    double corrected_angular_velocity_z = angular_velocity_z_ - gyro_bias_z_;

    // When the vehicle is stationary, a car-like robot should not rotate around z.
    // Also remove very small residual IMU noise after bias compensation.
    if (is_stationary || std::abs(corrected_angular_velocity_z) < gyro_deadband_) {
      corrected_angular_velocity_z = 0.0;
    }

    yaw_ += corrected_angular_velocity_z * dt;
    x_ += linear_velocity_ * std::cos(yaw_) * dt;
    y_ += linear_velocity_ * std::sin(yaw_) * dt;

    publish_odometry(now);
    last_update_time_ = now;
  }

  void publish_odometry(const rclcpp::Time & stamp)
  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = linear_velocity_;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_velocity_z_;

    // Simple diagonal covariance placeholders.
    odom_msg.pose.covariance[0] = 0.05;
    odom_msg.pose.covariance[7] = 0.05;
    odom_msg.pose.covariance[14] = 1e6;
    odom_msg.pose.covariance[21] = 1e6;
    odom_msg.pose.covariance[28] = 1e6;
    odom_msg.pose.covariance[35] = 0.2;

    odom_msg.twist.covariance[0] = 0.05;
    odom_msg.twist.covariance[7] = 0.05;
    odom_msg.twist.covariance[14] = 1e6;
    odom_msg.twist.covariance[21] = 1e6;
    odom_msg.twist.covariance[28] = 1e6;
    odom_msg.twist.covariance[35] = 0.2;

    odom_pub_->publish(odom_msg);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_link";
    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_update_time_;

  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
  double linear_velocity_ = 0.0;
  double angular_velocity_z_ = 0.0;

  double gyro_bias_z_ = 0.0;
  double stationary_speed_threshold_ = 0.01;  // m/s, use abs(v), so forward and reverse are handled the same way.
  double gyro_deadband_ = 0.003;              // rad/s, reject small residual gyro noise.
  double gyro_bias_alpha_ = 0.001;            // Slow low-pass update for gyro z bias while stationary.

  bool has_joint_ = false;
  bool has_imu_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QCar2Odometry>());
  rclcpp::shutdown();
  return 0;
}
