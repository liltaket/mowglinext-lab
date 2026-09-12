// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
/// @file gravity_estimator.hpp
/// @brief Conservative gravity-direction gate with stale-latch recovery.
//
// The scan ground filter needs a trustworthy gravity direction, but must not
// mistake a bump or body acceleration for gravity. Samples near the active
// gravity-magnitude baseline are accepted immediately. A plausible, but
// out-of-band, acceleration vector is held as a candidate; only five seconds
// of mutually-consistent full vectors
// can replace a stale baseline. This avoids a permanent rejected-sample latch
// while preserving pass-through behaviour until the new baseline is proven.

#ifndef MOWGLI_LOCALIZATION__GRAVITY_ESTIMATOR_HPP_
#define MOWGLI_LOCALIZATION__GRAVITY_ESTIMATOR_HPP_

#include <cmath>
#include <cstddef>

namespace mowgli_localization
{

constexpr double kStandardGravityMs2 = 9.80665;

struct GravityVector
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct GravityEstimatorConfig
{
  /// Allowed deviation from the active gravity baseline for ordinary samples.
  double accel_g_tolerance_ms2{3.0};
  /// Plausibility envelope applies even before the first accepted sample.
  double min_plausible_magnitude_ms2{0.5 * kStandardGravityMs2};
  double max_plausible_magnitude_ms2{1.5 * kStandardGravityMs2};
  /// Maximum full-vector distance within a stable rejected candidate run.
  double candidate_consistency_ms2{0.5};
  /// Stable candidate duration needed to replace a stale baseline.
  double reseed_after_s{5.0};
  /// Direction low-pass weight for ordinary accepted samples.
  double direction_alpha{0.2};
};

enum class GravityEstimatorAction
{
  SEEDED,
  ACCEPTED,
  REJECTED,
  RESEEDED,
  INVALID,
};

class GravityEstimator
{
public:
  GravityEstimator() = default;
  explicit GravityEstimator(const GravityEstimatorConfig& cfg) : cfg_(cfg)
  {
  }

  GravityEstimatorAction update(const GravityVector& acceleration, double t_s)
  {
    const double mag = magnitude(acceleration);
    if (!std::isfinite(t_s) || !finite(acceleration) || !std::isfinite(mag) || mag < 1e-3 ||
        mag < cfg_.min_plausible_magnitude_ms2 || mag > cfg_.max_plausible_magnitude_ms2)
    {
      clear_candidate();
      return GravityEstimatorAction::INVALID;
    }

    const GravityVector direction = normalized(acceleration, mag);
    if (std::abs(mag - baseline_magnitude_ms2_) <= cfg_.accel_g_tolerance_ms2)
    {
      clear_candidate();
      if (!have_direction_)
      {
        direction_ = direction;
        have_direction_ = true;
        return GravityEstimatorAction::SEEDED;
      }
      direction_ = normalized(add(scale(direction, cfg_.direction_alpha),
                                  scale(direction_, 1.0 - cfg_.direction_alpha)));
      return GravityEstimatorAction::ACCEPTED;
    }

    // Out of the normal band but physically plausible. It cannot refresh
    // freshness until a consistent, stationary-looking run proves a new bias.
    if (have_candidate_ &&
        distance(acceleration, candidate_anchor_) <= cfg_.candidate_consistency_ms2)
    {
      ++candidate_count_;
      candidate_mean_ = add(candidate_mean_,
                            scale(subtract(acceleration, candidate_mean_),
                                  1.0 / static_cast<double>(candidate_count_)));
      if (t_s >= candidate_start_s_ && t_s - candidate_start_s_ >= cfg_.reseed_after_s)
      {
        direction_ = normalized(candidate_mean_);
        baseline_magnitude_ms2_ = magnitude(candidate_mean_);
        have_direction_ = true;
        clear_candidate();
        return GravityEstimatorAction::RESEEDED;
      }
      return GravityEstimatorAction::REJECTED;
    }

    candidate_anchor_ = acceleration;
    candidate_mean_ = acceleration;
    candidate_count_ = 1;
    candidate_start_s_ = t_s;
    have_candidate_ = true;
    return GravityEstimatorAction::REJECTED;
  }

  bool has_direction() const
  {
    return have_direction_;
  }
  GravityVector direction() const
  {
    return direction_;
  }
  bool has_candidate() const
  {
    return have_candidate_;
  }
  double baseline_magnitude_ms2() const
  {
    return baseline_magnitude_ms2_;
  }

private:
  static bool finite(const GravityVector& v)
  {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
  }
  static double magnitude(const GravityVector& v)
  {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  }
  static GravityVector normalized(const GravityVector& v, double mag)
  {
    return GravityVector{v.x / mag, v.y / mag, v.z / mag};
  }
  static GravityVector normalized(const GravityVector& v)
  {
    return normalized(v, magnitude(v));
  }
  static GravityVector add(const GravityVector& a, const GravityVector& b)
  {
    return GravityVector{a.x + b.x, a.y + b.y, a.z + b.z};
  }
  static GravityVector subtract(const GravityVector& a, const GravityVector& b)
  {
    return GravityVector{a.x - b.x, a.y - b.y, a.z - b.z};
  }
  static GravityVector scale(const GravityVector& v, double factor)
  {
    return GravityVector{factor * v.x, factor * v.y, factor * v.z};
  }
  static double distance(const GravityVector& a, const GravityVector& b)
  {
    return magnitude(subtract(a, b));
  }
  void clear_candidate()
  {
    have_candidate_ = false;
    candidate_count_ = 0;
  }

  GravityEstimatorConfig cfg_{};
  bool have_direction_{false};
  GravityVector direction_{0.0, 0.0, 1.0};
  double baseline_magnitude_ms2_{kStandardGravityMs2};

  bool have_candidate_{false};
  GravityVector candidate_anchor_{};
  GravityVector candidate_mean_{};
  std::size_t candidate_count_{0};
  double candidate_start_s_{0.0};
};

}  // namespace mowgli_localization

#endif  // MOWGLI_LOCALIZATION__GRAVITY_ESTIMATOR_HPP_
