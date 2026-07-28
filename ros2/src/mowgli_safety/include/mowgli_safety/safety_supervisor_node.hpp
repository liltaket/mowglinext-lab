#pragma once

#include <deque>
#include <mutex>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mowgli_interfaces/msg/status.hpp"
#include "mowgli_interfaces/msg/emergency.hpp"
#include "mowgli_interfaces/srv/emergency_stop.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "mowgli_safety/safety_detector.hpp"

namespace mowgli_safety
{
class SafetySupervisorNode : public rclcpp::Node
{
public:
  SafetySupervisorNode();

private:
  void on_imu(sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void on_command(geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void on_status(mowgli_interfaces::msg::Status::ConstSharedPtr msg);
  void on_emergency(mowgli_interfaces::msg::Emergency::ConstSharedPtr msg);
  void on_timer();
  void request_emergency();
  void publish_diagnostics(const DetectorResult & result, const std::string & detail);
  rcl_interfaces::msg::SetParametersResult on_parameters(const std::vector<rclcpp::Parameter> & parameters);
  [[nodiscard]] bool fresh(const rclcpp::Time & time, double timeout_s) const;
  [[nodiscard]] double median(std::deque<double> values) const;

  mutable std::mutex mutex_;
  SafetyConfig config_;
  SafetyDetector detector_;
  bool enabled_{true}, shadow_mode_{true}, trip_on_imu_stale_when_active_{true};
  bool service_confirmed_{false}, request_in_flight_{false}, mower_active_{false}, charging_{false}, external_emergency_{false};
  double startup_grace_s_{5.0}, imu_timeout_s_{0.5}, odom_timeout_s_{0.5}, command_timeout_s_{0.5};
  double stable_clear_duration_s_{2.0}, gravity_tau_s_{0.5};
  rclcpp::Time started_, last_imu_, last_odom_, last_command_, last_status_, last_request_;
  sensor_msgs::msg::Imu::ConstSharedPtr imu_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr command_;
  mowgli_interfaces::msg::Status::ConstSharedPtr status_;
  std::deque<double> ax_, ay_, az_, gx_, gy_, gz_;
  double gravity_x_{0.0}, gravity_y_{0.0}, gravity_z_{9.80665}, previous_dynamic_norm_{0.0};
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_sub_;
  rclcpp::Subscription<mowgli_interfaces::msg::Status>::SharedPtr status_sub_;
  rclcpp::Subscription<mowgli_interfaces::msg::Emergency>::SharedPtr emergency_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr emergency_cmd_pub_;
  rclcpp::Client<mowgli_interfaces::srv::EmergencyStop>::SharedPtr emergency_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};
}  // namespace mowgli_safety
