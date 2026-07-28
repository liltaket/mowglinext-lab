#include "mowgli_safety/safety_supervisor_node.hpp"

#include <algorithm>
#include <cmath>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

namespace mowgli_safety
{
namespace
{
constexpr double kGravity = 9.80665;
constexpr double kSafetyDiagnosticsPeriodS = 0.2;
bool finite(double value) { return std::isfinite(value); }
std::string state_name(SafetyState state) {
  return state == SafetyState::TRIPPED ? "TRIPPED" : state == SafetyState::SUSPECT ? "SUSPECT" : "NORMAL";
}
std::string trip_name(TripType type) {
  return type == TripType::ROLLOVER ? "ROLLOVER" : type == TripType::IMPACT ? "IMPACT" : type == TripType::IMU_STALE ? "IMU_STALE" : "NONE";
}
}  // namespace

SafetySupervisorNode::SafetySupervisorNode() : Node("safety_supervisor"), detector_(config_)
{
  started_ = now();
  enabled_ = declare_parameter("enabled", true);
  shadow_mode_ = declare_parameter("shadow_mode", true);
  startup_grace_s_ = declare_parameter("startup_grace_sec", 5.0);
  imu_timeout_s_ = declare_parameter("imu_timeout_sec", 0.5);
  odom_timeout_s_ = declare_parameter("odom_timeout_sec", 0.5);
  command_timeout_s_ = declare_parameter("command_timeout_sec", 0.5);
  stable_clear_duration_s_ = declare_parameter("stable_clear_duration_sec", 2.0);
  trip_on_imu_stale_when_active_ = declare_parameter("trip_on_imu_stale_when_active", true);
  config_.command_stop_grace_s = declare_parameter("command_stop_grace_ms", 500) / 1000.0;
  config_.absolute_trip_deg = declare_parameter("tilt.absolute_trip_deg", 48.0);
  config_.absolute_trip_duration_s = declare_parameter("tilt.absolute_trip_duration_ms", 250) / 1000.0;
  config_.hard_absolute_trip_deg = declare_parameter("tilt.hard_absolute_trip_deg", 68.0);
  config_.hard_absolute_trip_duration_s = declare_parameter("tilt.hard_absolute_trip_duration_ms", 100) / 1000.0;
  config_.relative_trip_deg = declare_parameter("tilt.relative_trip_deg", 30.0);
  config_.relative_trip_duration_s = declare_parameter("tilt.relative_trip_duration_ms", 350) / 1000.0;
  config_.rapid_tilt_min_deg = declare_parameter("tilt.rapid_tilt_min_deg", 32.0);
  config_.rapid_tilt_rate_deg_s = declare_parameter("tilt.rapid_tilt_rate_deg_s", 110.0);
  config_.rapid_tilt_confirm_s = declare_parameter("tilt.rapid_tilt_confirm_ms", 300) / 1000.0;
  config_.clear_deg = declare_parameter("tilt.clear_deg", 22.0);
  config_.terrain_baseline_tau_s = declare_parameter("tilt.terrain_baseline_tau_sec", 4.0);
  config_.terrain_baseline_max_deg = declare_parameter("tilt.terrain_baseline_max_deg", 32.0);
  config_.horizontal_accel_trip_mps2 = declare_parameter("impact.horizontal_accel_trip_mps2", 9.0);
  config_.jerk_trip_mps3 = declare_parameter("impact.jerk_trip_mps3", 55.0);
  config_.gyro_trip_rad_s = declare_parameter("impact.gyro_trip_rad_s", 2.0);
  config_.min_commanded_speed_mps = declare_parameter("impact.min_commanded_speed_mps", 0.08);
  config_.min_actual_speed_before_mps = declare_parameter("impact.min_actual_speed_before_mps", 0.07);
  config_.speed_drop_fraction = declare_parameter("impact.speed_drop_fraction", 0.70);
  config_.speed_drop_window_s = declare_parameter("impact.speed_drop_window_ms", 300) / 1000.0;
  config_.confirmation_window_s = declare_parameter("impact.confirmation_window_ms", 300) / 1000.0;
  config_.required_evidence_count = declare_parameter("impact.required_evidence_count", 2);
  declare_parameter("impact.recovery_window_ms", 450); declare_parameter("impact.recovered_speed_fraction", 0.65);
  declare_parameter("filters.imu_median_samples", 5); gravity_tau_s_ = declare_parameter("filters.gravity_tau_sec", 0.5);
  declare_parameter("filters.wheel_speed_tau_sec", 0.12);
  detector_.set_config(config_);
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu/data", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::Imu::ConstSharedPtr message) { on_imu(message); });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/wheel_odom", 20,
    [this](nav_msgs::msg::Odometry::ConstSharedPtr message) { on_odom(message); });
  command_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("/cmd_vel", 20,
    [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr message) { on_command(message); });
  status_sub_ = create_subscription<mowgli_interfaces::msg::Status>("/hardware_bridge/status", 10,
    [this](mowgli_interfaces::msg::Status::ConstSharedPtr message) { on_status(message); });
  emergency_sub_ = create_subscription<mowgli_interfaces::msg::Emergency>("/hardware_bridge/emergency", 10,
    [this](mowgli_interfaces::msg::Emergency::ConstSharedPtr message) { on_emergency(message); });
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  safety_diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/safety_supervisor/diagnostics", 10);
  emergency_cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_emergency", 10);
  emergency_client_ = create_client<mowgli_interfaces::srv::EmergencyStop>("/hardware_bridge/emergency_stop");
  parameter_callback_ = add_on_set_parameters_callback([this](const auto & p) { return on_parameters(p); });
  timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] { on_timer(); });
}

double SafetySupervisorNode::median(std::deque<double> values) const { std::sort(values.begin(), values.end()); return values[values.size() / 2]; }
bool SafetySupervisorNode::fresh(const rclcpp::Time & time, double timeout_s) const { return time.nanoseconds() > 0 && (now() - time).seconds() <= timeout_s; }
void SafetySupervisorNode::on_imu(sensor_msgs::msg::Imu::ConstSharedPtr msg) { std::lock_guard<std::mutex> lock(mutex_); imu_ = msg; last_imu_ = now(); }
void SafetySupervisorNode::on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg) { std::lock_guard<std::mutex> lock(mutex_); odom_ = msg; last_odom_ = now(); }
void SafetySupervisorNode::on_command(geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) { std::lock_guard<std::mutex> lock(mutex_); command_ = msg; last_command_ = now(); }
void SafetySupervisorNode::on_status(mowgli_interfaces::msg::Status::ConstSharedPtr msg) { std::lock_guard<std::mutex> lock(mutex_); status_ = msg; charging_ = msg->is_charging; mower_active_ = msg->mow_enabled || (!charging_ && command_ && std::abs(command_->twist.linear.x) > 0.02); last_status_ = now(); }
void SafetySupervisorNode::on_emergency(mowgli_interfaces::msg::Emergency::ConstSharedPtr msg) { std::lock_guard<std::mutex> lock(mutex_); external_emergency_ = msg->active_emergency || msg->latched_emergency; }

void SafetySupervisorNode::on_timer()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto t = now();
  if (!enabled_ || (t - started_).seconds() < startup_grace_s_) return;
  const bool imu_fresh = fresh(last_imu_, imu_timeout_s_), odom_fresh = fresh(last_odom_, odom_timeout_s_), command_fresh = fresh(last_command_, command_timeout_s_);
  if ((!imu_fresh || !odom_fresh || !command_fresh) && mower_active_) {
    DetectorResult stale{SafetyState::SUSPECT, TripType::NONE, 0.0, 0, "stale sensor"};
    if (!imu_fresh && trip_on_imu_stale_when_active_) stale = {SafetyState::TRIPPED, TripType::IMU_STALE, 0.0, 0, "USB IMU stale while active"};
    publish_diagnostics(stale, stale.reason); if (stale.state == SafetyState::TRIPPED && !shadow_mode_) request_emergency(); return;
  }
  if (!imu_ || !odom_ || !command_ || !status_ || !imu_fresh) return;
  const auto & a = imu_->linear_acceleration; const auto & g = imu_->angular_velocity;
  const auto & q = imu_->orientation;
  const bool valid = finite(a.x) && finite(a.y) && finite(a.z) && finite(g.x) && finite(g.y) && finite(g.z);
  if (!valid) { publish_diagnostics({SafetyState::SUSPECT, TripType::NONE, 0, 0, "invalid IMU values"}, "invalid IMU values"); return; }
  const int sample_count = get_parameter("filters.imu_median_samples").as_int();
  auto add = [sample_count](std::deque<double> & values, double value) { values.push_back(value); while (static_cast<int>(values.size()) > sample_count) values.pop_front(); };
  add(ax_, a.x); add(ay_, a.y); add(az_, a.z); add(gx_, g.x); add(gy_, g.y); add(gz_, g.z);
  const double mx = median(ax_), my = median(ay_), mz = median(az_); const double dt = 0.05;
  const double alpha = dt / (gravity_tau_s_ + dt); gravity_x_ += alpha * (mx - gravity_x_); gravity_y_ += alpha * (my - gravity_y_); gravity_z_ += alpha * (mz - gravity_z_);
  const double dx = mx - gravity_x_, dy = my - gravity_y_, dz = mz - gravity_z_;
  const double dynamic_norm = std::sqrt(dx * dx + dy * dy + dz * dz); const double jerk = std::abs(dynamic_norm - previous_dynamic_norm_) / dt; previous_dynamic_norm_ = dynamic_norm;
  const double qnorm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
  double roll = std::atan2(gravity_y_, gravity_z_), pitch = std::atan2(-gravity_x_, std::hypot(gravity_y_, gravity_z_));
  if (finite(qnorm) && qnorm > 0.5 && qnorm < 1.5 && imu_->orientation_covariance[0] >= 0.0) {
    roll = std::atan2(2.0*(q.w*q.x + q.y*q.z), 1.0 - 2.0*(q.x*q.x + q.y*q.y));
    pitch = std::asin(std::clamp(2.0*(q.w*q.y - q.z*q.x), -1.0, 1.0));
  }
  DetectorInput input{t.seconds(), std::hypot(roll, pitch) * 180.0 / M_PI, g.x * 180.0 / M_PI, g.y * 180.0 / M_PI,
    std::hypot(dx, dy), jerk, std::sqrt(g.x*g.x + g.y*g.y + g.z*g.z), odom_->twist.twist.linear.x, command_->twist.linear.x,
    true, mower_active_, charging_};
  const auto result = detector_.update(input);
  const bool safe_and_still = input.absolute_tilt_deg < config_.clear_deg &&
                              std::abs(input.actual_speed_mps) < 0.02 &&
                              std::abs(input.commanded_speed_mps) < 0.02;
  detector_.reset_if_safe(external_emergency_, safe_and_still, t.seconds(), stable_clear_duration_s_);
  if (detector_.state() == SafetyState::NORMAL && !external_emergency_) service_confirmed_ = false;
  publish_diagnostics(result, result.reason);
  if (result.state == SafetyState::TRIPPED) { RCLCPP_ERROR(get_logger(), "Safety trip: %s (%s)", trip_name(result.trip).c_str(), result.reason.c_str()); if (!shadow_mode_) request_emergency(); }
}

void SafetySupervisorNode::request_emergency()
{
  if (service_confirmed_ || request_in_flight_ || (now() - last_request_).seconds() < 0.1) return;
  geometry_msgs::msg::TwistStamped zero; zero.header.stamp = now(); emergency_cmd_pub_->publish(zero);
  last_request_ = now();
  if (!emergency_client_->service_is_ready()) { RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Emergency service unavailable; retrying"); return; }
  request_in_flight_ = true; auto request = std::make_shared<mowgli_interfaces::srv::EmergencyStop::Request>(); request->emergency = 1;
  emergency_client_->async_send_request(request,
    [this](rclcpp::Client<mowgli_interfaces::srv::EmergencyStop>::SharedFuture future) {
      std::lock_guard<std::mutex> lock(mutex_);
      request_in_flight_ = false;
      service_confirmed_ = future.get()->success;
    });
}

void SafetySupervisorNode::publish_diagnostics(const DetectorResult & result, const std::string & detail)
{
  diagnostic_msgs::msg::DiagnosticStatus status; status.name = "Safety Supervisor"; status.hardware_id = "mowgli/safety_supervisor";
  status.level = result.state == SafetyState::TRIPPED ? diagnostic_msgs::msg::DiagnosticStatus::ERROR : (shadow_mode_ || result.state == SafetyState::SUSPECT ? diagnostic_msgs::msg::DiagnosticStatus::WARN : diagnostic_msgs::msg::DiagnosticStatus::OK);
  status.message = detail;
  for (const auto & item : std::vector<std::pair<std::string, std::string>>{{"state", state_name(result.state)}, {"shadow_mode", shadow_mode_ ? "true" : "false"}, {"trip", trip_name(result.trip)}, {"impact_evidence_count", std::to_string(result.impact_evidence_count)}, {"emergency_request_confirmed", service_confirmed_ ? "true" : "false"}}) { diagnostic_msgs::msg::KeyValue value; value.key = item.first; value.value = item.second; status.values.push_back(value); }
  diagnostic_msgs::msg::DiagnosticArray array; array.header.stamp = now(); array.status.push_back(status); diagnostics_pub_->publish(array);
  const bool state_changed = result.state != last_safety_diagnostics_state_ ||
                             result.trip != last_safety_diagnostics_trip_ ||
                             detail != last_safety_diagnostics_detail_;
  const bool rate_due = last_safety_diagnostics_publish_.nanoseconds() == 0 ||
                        (array.header.stamp - last_safety_diagnostics_publish_).seconds() >=
                          kSafetyDiagnosticsPeriodS;
  if (state_changed || rate_due) {
    safety_diagnostics_pub_->publish(array);
    last_safety_diagnostics_publish_ = array.header.stamp;
    last_safety_diagnostics_state_ = result.state;
    last_safety_diagnostics_trip_ = result.trip;
    last_safety_diagnostics_detail_ = detail;
  }
}

rcl_interfaces::msg::SetParametersResult SafetySupervisorNode::on_parameters(const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto proposed = config_; bool enabled = enabled_, shadow = shadow_mode_;
  for (const auto & p : parameters) {
    const auto n = p.get_name(); if (n == "enabled") enabled = p.as_bool(); else if (n == "shadow_mode") shadow = p.as_bool();
    else if (n == "tilt.absolute_trip_deg") proposed.absolute_trip_deg = p.as_double();
    else if (n == "tilt.hard_absolute_trip_deg") proposed.hard_absolute_trip_deg = p.as_double();
    else if (n == "tilt.relative_trip_deg") proposed.relative_trip_deg = p.as_double();
    else if (n == "tilt.rapid_tilt_min_deg") proposed.rapid_tilt_min_deg = p.as_double();
    else if (n == "tilt.rapid_tilt_rate_deg_s") proposed.rapid_tilt_rate_deg_s = p.as_double();
    else if (n == "tilt.clear_deg") proposed.clear_deg = p.as_double();
    else if (n == "tilt.terrain_baseline_tau_sec") proposed.terrain_baseline_tau_s = p.as_double();
    else if (n == "tilt.terrain_baseline_max_deg") proposed.terrain_baseline_max_deg = p.as_double();
    else if (n == "tilt.absolute_trip_duration_ms") proposed.absolute_trip_duration_s = p.as_int() / 1000.0;
    else if (n == "tilt.hard_absolute_trip_duration_ms") proposed.hard_absolute_trip_duration_s = p.as_int() / 1000.0;
    else if (n == "tilt.relative_trip_duration_ms") proposed.relative_trip_duration_s = p.as_int() / 1000.0;
    else if (n == "tilt.rapid_tilt_confirm_ms") proposed.rapid_tilt_confirm_s = p.as_int() / 1000.0;
    else if (n == "impact.horizontal_accel_trip_mps2") proposed.horizontal_accel_trip_mps2 = p.as_double();
    else if (n == "impact.jerk_trip_mps3") proposed.jerk_trip_mps3 = p.as_double();
    else if (n == "impact.gyro_trip_rad_s") proposed.gyro_trip_rad_s = p.as_double();
    else if (n == "impact.min_commanded_speed_mps") proposed.min_commanded_speed_mps = p.as_double();
    else if (n == "impact.min_actual_speed_before_mps") proposed.min_actual_speed_before_mps = p.as_double();
    else if (n == "impact.speed_drop_fraction") proposed.speed_drop_fraction = p.as_double();
    else if (n == "impact.speed_drop_window_ms") proposed.speed_drop_window_s = p.as_int() / 1000.0;
    else if (n == "impact.confirmation_window_ms") proposed.confirmation_window_s = p.as_int() / 1000.0;
    else if (n == "impact.required_evidence_count") proposed.required_evidence_count = p.as_int();
    else if (n == "command_stop_grace_ms") proposed.command_stop_grace_s = p.as_int() / 1000.0;
  }
  rcl_interfaces::msg::SetParametersResult result; result.successful = proposed.clear_deg > 0.0 && proposed.clear_deg < proposed.absolute_trip_deg && proposed.absolute_trip_deg < 90.0 && proposed.hard_absolute_trip_deg > proposed.absolute_trip_deg && proposed.relative_trip_deg > 0.0 && proposed.speed_drop_fraction > 0.0 && proposed.speed_drop_fraction < 1.0 && proposed.required_evidence_count >= 1 && proposed.required_evidence_count <= 3 && proposed.absolute_trip_duration_s > 0.0 && proposed.terrain_baseline_tau_s > 0.0;
  if (!result.successful) { result.reason = "invalid safety thresholds: ensure clear < trip, degrees are 0..90 and evidence is 1..3"; return result; }
  config_ = proposed; detector_.set_config(config_); enabled_ = enabled; shadow_mode_ = shadow; RCLCPP_INFO(get_logger(), "Safety parameters applied atomically (shadow_mode=%s)", shadow_mode_ ? "true" : "false"); return result;
}
}  // namespace mowgli_safety
