#pragma once

#include <string>

namespace mowgli_safety
{
enum class SafetyState { NORMAL, SUSPECT, TRIPPED };
enum class TripType { NONE, ROLLOVER, IMPACT, STALL, IMU_STALE };

struct SafetyConfig
{
  double absolute_trip_deg{48.0}, hard_absolute_trip_deg{68.0}, relative_trip_deg{30.0};
  double rapid_tilt_min_deg{32.0}, rapid_tilt_rate_deg_s{110.0}, clear_deg{22.0};
  double absolute_trip_duration_s{0.25}, hard_absolute_trip_duration_s{0.10};
  double relative_trip_duration_s{0.35}, rapid_tilt_confirm_s{0.30};
  double terrain_baseline_tau_s{4.0}, terrain_baseline_max_deg{32.0};
  double horizontal_accel_trip_mps2{9.0}, jerk_trip_mps3{55.0}, gyro_trip_rad_s{2.0};
  double min_commanded_speed_mps{0.08}, min_actual_speed_before_mps{0.07};
  double speed_drop_fraction{0.70}, speed_drop_window_s{0.30}, confirmation_window_s{0.30};
  int required_evidence_count{2};
  double command_stop_grace_s{0.50};
};

struct DetectorInput
{
  double stamp_s{0.0};
  double absolute_tilt_deg{0.0}, roll_rate_deg_s{0.0}, pitch_rate_deg_s{0.0};
  double horizontal_accel_mps2{0.0}, jerk_mps3{0.0}, gyro_norm_rad_s{0.0};
  double actual_speed_mps{0.0}, commanded_speed_mps{0.0};
  bool imu_valid{false}, active{false}, charging{false};
};

struct DetectorResult
{
  SafetyState state{SafetyState::NORMAL};
  TripType trip{TripType::NONE};
  double relative_tilt_deg{0.0};
  int impact_evidence_count{0};
  std::string reason;
};

class SafetyDetector
{
public:
  explicit SafetyDetector(SafetyConfig config = {});
  void set_config(const SafetyConfig & config);
  DetectorResult update(const DetectorInput & input);
  DetectorResult force_trip(const DetectorInput & input, TripType type, const std::string & reason);
  void reset_if_safe(bool external_emergency_active, bool safe_and_still, double stamp_s,
    double stable_clear_duration_s);
  [[nodiscard]] SafetyState state() const { return state_; }

private:
  SafetyConfig config_;
  SafetyState state_{SafetyState::NORMAL};
  double baseline_deg_{0.0}, last_stamp_s_{-1.0}, last_actual_speed_{0.0};
  double absolute_since_s_{-1.0}, hard_since_s_{-1.0}, relative_since_s_{-1.0}, rapid_since_s_{-1.0};
  double evidence_a_s_{-1.0}, evidence_b_s_{-1.0}, evidence_c_s_{-1.0}, command_stop_s_{-1.0};
  double safe_since_s_{-1.0};
  TripType trip_{TripType::NONE};
  DetectorResult trip_result(const DetectorInput & input, TripType type, const std::string & reason);
};
}  // namespace mowgli_safety
