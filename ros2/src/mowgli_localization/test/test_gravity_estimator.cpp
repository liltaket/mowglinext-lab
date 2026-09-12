// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "mowgli_localization/gravity_estimator.hpp"

using mowgli_localization::GravityEstimator;
using mowgli_localization::GravityEstimatorAction;
using mowgli_localization::GravityVector;
using mowgli_localization::kStandardGravityMs2;

namespace
{
double magnitude(const GravityVector& v)
{
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}
}  // namespace

TEST(GravityEstimator, NormalGravityAndSmallTiltAreAccepted)
{
  GravityEstimator estimator;
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, kStandardGravityMs2}, 0.0),
            GravityEstimatorAction::SEEDED);
  EXPECT_EQ(estimator.update(GravityVector{-1.7, 0.0, 9.66}, 0.1),
            GravityEstimatorAction::ACCEPTED);
  EXPECT_TRUE(estimator.has_direction());
  EXPECT_NEAR(magnitude(estimator.direction()), 1.0, 1e-12);
  EXPECT_LT(estimator.direction().x, 0.0);
}

TEST(GravityEstimator, SingleTransientDoesNotReplaceBaseline)
{
  GravityEstimator estimator;
  estimator.update(GravityVector{0.0, 0.0, kStandardGravityMs2}, 0.0);
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 0.1),
            GravityEstimatorAction::REJECTED);
  EXPECT_TRUE(estimator.has_candidate());
  EXPECT_NEAR(estimator.direction().z, 1.0, 1e-12);
}

TEST(GravityEstimator, StartupOffsetReseedsBeforeBecomingUsable)
{
  GravityEstimator estimator;
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 0.1),
            GravityEstimatorAction::REJECTED);
  EXPECT_FALSE(estimator.has_direction());
  for (int i = 1; i < 50; ++i)
  {
    EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 0.1 + i * 0.1),
              GravityEstimatorAction::REJECTED);
  }
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 5.1),
            GravityEstimatorAction::RESEEDED);
  EXPECT_TRUE(estimator.has_direction());
  EXPECT_NEAR(estimator.baseline_magnitude_ms2(), 13.09, 1e-12);
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 5.2),
            GravityEstimatorAction::ACCEPTED);
}

TEST(GravityEstimator, StableOffsetReseedsAndContinuesAcceptingNewBaseline)
{
  GravityEstimator estimator;
  estimator.update(GravityVector{0.0, 0.0, kStandardGravityMs2}, 0.0);
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 0.1),
            GravityEstimatorAction::REJECTED);
  for (int i = 1; i < 50; ++i)
  {
    EXPECT_EQ(estimator.update(GravityVector{0.1, -0.1, 13.09}, 0.1 + i * 0.1),
              GravityEstimatorAction::REJECTED);
  }
  EXPECT_EQ(estimator.update(GravityVector{0.1, -0.1, 13.09}, 5.1),
            GravityEstimatorAction::RESEEDED);
  // The first candidate is exactly vertical; re-seed uses the full candidate
  // average rather than the final sample's direction.
  const double mean_x = 50.0 * 0.1 / 51.0;
  const double mean_y = -mean_x;
  EXPECT_NEAR(estimator.direction().x,
              mean_x / std::sqrt(mean_x * mean_x + mean_y * mean_y + 13.09 * 13.09),
              1e-12);
  EXPECT_NEAR(magnitude(estimator.direction()), 1.0, 1e-12);
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 5.2),
            GravityEstimatorAction::ACCEPTED);
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 5.3),
            GravityEstimatorAction::ACCEPTED);
  EXPECT_TRUE(estimator.has_direction());
  EXPECT_NEAR(magnitude(estimator.direction()), 1.0, 1e-12);
}

TEST(GravityEstimator, CandidateInconsistencyPreventsReseed)
{
  GravityEstimator estimator;
  estimator.update(GravityVector{0.0, 0.0, kStandardGravityMs2}, 0.0);
  for (int i = 1; i <= 70; ++i)
  {
    const GravityVector sample =
        (i % 2 == 0) ? GravityVector{0.0, 0.0, 13.09} : GravityVector{1.0, 0.0, 13.09};
    EXPECT_EQ(estimator.update(sample, i * 0.1), GravityEstimatorAction::REJECTED);
  }
  EXPECT_NEAR(estimator.direction().z, 1.0, 1e-12);
}

TEST(GravityEstimator, SlowlyDriftingCandidateCannotFollowItsOwnMean)
{
  GravityEstimator estimator;
  estimator.update(GravityVector{0.0, 0.0, kStandardGravityMs2}, 0.0);
  for (int i = 0; i <= 60; ++i)
  {
    const double x = 0.02 * i;
    EXPECT_EQ(estimator.update(GravityVector{x, 0.0, 13.09}, 0.1 * i),
              GravityEstimatorAction::REJECTED);
  }
  EXPECT_NEAR(estimator.baseline_magnitude_ms2(), kStandardGravityMs2, 1e-12);
}

TEST(GravityEstimator, ObviousMotionAndInvalidSamplesAreRejectedAndResetCandidate)
{
  GravityEstimator estimator;
  estimator.update(GravityVector{0.0, 0.0, kStandardGravityMs2}, 0.0);
  estimator.update(GravityVector{0.0, 0.0, 13.09}, 0.1);
  EXPECT_TRUE(estimator.has_candidate());
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 20.0}, 0.2), GravityEstimatorAction::INVALID);
  EXPECT_FALSE(estimator.has_candidate());
  EXPECT_EQ(estimator.update(GravityVector{std::numeric_limits<double>::quiet_NaN(), 0.0, 9.8},
                             0.3),
            GravityEstimatorAction::INVALID);
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 0.0}, 0.4), GravityEstimatorAction::INVALID);
}

TEST(GravityEstimator, AcceptedSampleClearsRejectedRunAndRecoveryWorks)
{
  GravityEstimator estimator;
  estimator.update(GravityVector{0.0, 0.0, kStandardGravityMs2}, 0.0);
  estimator.update(GravityVector{0.0, 0.0, 13.09}, 0.1);
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, kStandardGravityMs2}, 0.2),
            GravityEstimatorAction::ACCEPTED);
  EXPECT_FALSE(estimator.has_candidate());

  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 0.3),
            GravityEstimatorAction::REJECTED);
  GravityEstimatorAction action = GravityEstimatorAction::REJECTED;
  for (int i = 1; i <= 50; ++i)
    action = estimator.update(GravityVector{0.0, 0.0, 13.09}, 0.3 + i * 0.1);
  EXPECT_EQ(action, GravityEstimatorAction::RESEEDED);
}
