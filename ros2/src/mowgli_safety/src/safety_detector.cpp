#include "mowgli_safety/safety_detector.hpp"

#include <algorithm>
#include <cmath>

namespace mowgli_safety
{
SafetyDetector::SafetyDetector(SafetyConfig config) : config_(config) {}
void SafetyDetector::set_config(const SafetyConfig & config) { config_ = config; }

DetectorResult SafetyDetector::trip_result(const DetectorInput & input, TripType type,
  const std::string & reason)
{
  state_ = SafetyState::TRIPPED;
  trip_ = type;
  return {state_, type, input.absolute_tilt_deg - baseline_deg_, 0, reason};
}

DetectorResult SafetyDetector::update(const DetectorInput & in)
{
  if (state_ == SafetyState::TRIPPED) {
    return {state_, trip_, in.absolute_tilt_deg - baseline_deg_, 0, "latched"};
  }
  if (!in.imu_valid || !std::isfinite(in.stamp_s)) {
    return {SafetyState::SUSPECT, TripType::NONE, 0.0, 0, "invalid imu"};
  }
  const double dt = last_stamp_s_ < 0.0 ? 0.0 : in.stamp_s - last_stamp_s_;
  if (dt < 0.0 || dt > 1.0) { baseline_deg_ = in.absolute_tilt_deg; }
  last_stamp_s_ = in.stamp_s;
  const double tilt_rate = std::max(std::abs(in.roll_rate_deg_s), std::abs(in.pitch_rate_deg_s));
  if (dt > 0.0 && tilt_rate < 15.0 && std::abs(in.commanded_speed_mps) < 0.08 &&
      in.absolute_tilt_deg < config_.terrain_baseline_max_deg) {
    const double alpha = dt / (config_.terrain_baseline_tau_s + dt);
    baseline_deg_ += alpha * (in.absolute_tilt_deg - baseline_deg_);
  }
  const double relative = std::abs(in.absolute_tilt_deg - baseline_deg_);
  auto held = [t = in.stamp_s](bool condition, double & since, double duration) {
    if (!condition) { since = -1.0; return false; }
    if (since < 0.0) { since = t; }
    return t - since >= duration;
  };
  if (in.active && !in.charging) {
    if (held(in.absolute_tilt_deg >= config_.hard_absolute_trip_deg, hard_since_s_, config_.hard_absolute_trip_duration_s))
      return trip_result(in, TripType::ROLLOVER, "hard absolute tilt");
    if (held(in.absolute_tilt_deg >= config_.absolute_trip_deg, absolute_since_s_, config_.absolute_trip_duration_s))
      return trip_result(in, TripType::ROLLOVER, "absolute tilt");
    if (held(relative >= config_.relative_trip_deg, relative_since_s_, config_.relative_trip_duration_s))
      return trip_result(in, TripType::ROLLOVER, "relative tilt");
    if (held(in.absolute_tilt_deg >= config_.rapid_tilt_min_deg && tilt_rate >= config_.rapid_tilt_rate_deg_s,
        rapid_since_s_, config_.rapid_tilt_confirm_s)) return trip_result(in, TripType::ROLLOVER, "rapid tilt");
  }
  if (std::abs(in.commanded_speed_mps) < config_.min_commanded_speed_mps) command_stop_s_ = in.stamp_s;
  const bool command_grace = command_stop_s_ >= 0.0 && in.stamp_s - command_stop_s_ < config_.command_stop_grace_s;
  if (in.active && !command_grace) {
    if (in.horizontal_accel_mps2 >= config_.horizontal_accel_trip_mps2 || in.jerk_mps3 >= config_.jerk_trip_mps3) evidence_a_s_ = in.stamp_s;
    if (in.gyro_norm_rad_s >= config_.gyro_trip_rad_s) evidence_c_s_ = in.stamp_s;
    const bool speed_drop = std::abs(in.commanded_speed_mps) >= config_.min_commanded_speed_mps &&
      std::abs(last_actual_speed_) >= config_.min_actual_speed_before_mps &&
      std::abs(in.actual_speed_mps) <= std::abs(last_actual_speed_) * (1.0 - config_.speed_drop_fraction);
    if (speed_drop) evidence_b_s_ = in.stamp_s;
    int evidence = 0;
    for (const double stamp : {evidence_a_s_, evidence_b_s_, evidence_c_s_})
      if (stamp >= 0.0 && in.stamp_s - stamp <= config_.confirmation_window_s) ++evidence;
    last_actual_speed_ = in.actual_speed_mps;
    if (evidence >= config_.required_evidence_count)
      return trip_result(in, TripType::IMPACT, "corroborated impact evidence");
    state_ = evidence ? SafetyState::SUSPECT : SafetyState::NORMAL;
    return {state_, TripType::NONE, relative, evidence, evidence ? "impact candidate" : "normal"};
  }
  last_actual_speed_ = in.actual_speed_mps;
  state_ = SafetyState::NORMAL;
  return {state_, TripType::NONE, relative, 0, "normal"};
}

void SafetyDetector::reset_if_safe(bool external_emergency_active, bool safe_and_still, double stamp_s,
  double stable_clear_duration_s)
{
  if (external_emergency_active || !safe_and_still) { safe_since_s_ = -1.0; return; }
  if (safe_since_s_ < 0.0) safe_since_s_ = stamp_s;
  if (state_ == SafetyState::TRIPPED && stamp_s - safe_since_s_ >= stable_clear_duration_s) {
    state_ = SafetyState::NORMAL; trip_ = TripType::NONE;
  }
}
}  // namespace mowgli_safety
