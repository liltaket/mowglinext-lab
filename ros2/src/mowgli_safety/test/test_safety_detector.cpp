#include <gtest/gtest.h>
#include "mowgli_safety/safety_detector.hpp"
using mowgli_safety::DetectorInput; using mowgli_safety::SafetyDetector; using mowgli_safety::SafetyState; using mowgli_safety::TripType;
namespace { DetectorInput sample(double t) { DetectorInput x; x.stamp_s=t; x.imu_valid=true; x.active=true; x.actual_speed_mps=.2; x.commanded_speed_mps=.2; return x; } }
TEST(SafetyDetector, FlatGroundDoesNotTrip) { SafetyDetector d; for (int i=0;i<100;++i) EXPECT_NE(d.update(sample(i*.05)).state, SafetyState::TRIPPED); }
TEST(SafetyDetector, SustainedAbsoluteTiltTrips) { SafetyDetector d; for(int i=0;i<8;++i) { auto x=sample(i*.05); x.absolute_tilt_deg=55; if(i==7) EXPECT_EQ(d.update(x).trip, TripType::ROLLOVER); else d.update(x); } }
TEST(SafetyDetector, CommandedStopSuppressesImpact) { SafetyDetector d; auto moving=sample(0); d.update(moving); auto stop=sample(.1); stop.commanded_speed_mps=0; stop.actual_speed_mps=0; stop.horizontal_accel_mps2=20; stop.gyro_norm_rad_s=3; EXPECT_NE(d.update(stop).state, SafetyState::TRIPPED); }
TEST(SafetyDetector, CorroboratedImpactTrips) { SafetyDetector d; d.update(sample(0)); auto hit=sample(.1); hit.actual_speed_mps=.01; hit.horizontal_accel_mps2=15; EXPECT_EQ(d.update(hit).trip, TripType::IMPACT); }
TEST(SafetyDetector, ChargingTiltDoesNotTrip) { SafetyDetector d; for(int i=0;i<20;++i) { auto x=sample(i*.05); x.charging=true; x.absolute_tilt_deg=60; EXPECT_NE(d.update(x).state, SafetyState::TRIPPED); } }
