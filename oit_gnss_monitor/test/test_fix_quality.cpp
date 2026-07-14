#include <gtest/gtest.h>
#include <limits>

#include "oit_gnss_monitor/fix_quality.hpp"

namespace {
using oit_gnss_monitor::QualityLimits;
using oit_gnss_monitor::QualityState;
using oit_gnss_monitor::evaluate_fix;
constexpr int64_t kNow = 101000000000LL;

sensor_msgs::msg::NavSatFix valid_fix() {
  sensor_msgs::msg::NavSatFix fix;
  fix.header.stamp.sec = 100;
  fix.latitude = 35.0;
  fix.longitude = 139.0;
  fix.altitude = 10.0;
  fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  fix.position_covariance = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  return fix;
}

TEST(FixQuality, AcceptsValidFix) { EXPECT_TRUE(evaluate_fix(valid_fix(), kNow, {}, {}).accepted); }
TEST(FixQuality, RejectsNaNLatitude) { auto f=valid_fix(); f.latitude=std::numeric_limits<double>::quiet_NaN(); EXPECT_FALSE(evaluate_fix(f,kNow,{},{}).accepted); }
TEST(FixQuality, RejectsInfiniteLongitude) { auto f=valid_fix(); f.longitude=std::numeric_limits<double>::infinity(); EXPECT_FALSE(evaluate_fix(f,kNow,{},{}).accepted); }
TEST(FixQuality, RejectsLatitudeAboveRange) { auto f=valid_fix(); f.latitude=90.1; EXPECT_FALSE(evaluate_fix(f,kNow,{},{}).accepted); }
TEST(FixQuality, RejectsLatitudeBelowRange) { auto f=valid_fix(); f.latitude=-90.1; EXPECT_FALSE(evaluate_fix(f,kNow,{},{}).accepted); }
TEST(FixQuality, RejectsLongitudeAboveRange) { auto f=valid_fix(); f.longitude=180.1; EXPECT_FALSE(evaluate_fix(f,kNow,{},{}).accepted); }
TEST(FixQuality, RejectsLongitudeBelowRange) { auto f=valid_fix(); f.longitude=-180.1; EXPECT_FALSE(evaluate_fix(f,kNow,{},{}).accepted); }
TEST(FixQuality, RejectsNoFix) { auto f=valid_fix(); f.status.status=sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX; EXPECT_FALSE(evaluate_fix(f,kNow,{},{}).accepted); }
TEST(FixQuality, RejectsZeroStamp) { auto f=valid_fix(); f.header.stamp.sec=0; EXPECT_FALSE(evaluate_fix(f,kNow,{},{}).accepted); }
TEST(FixQuality, RejectsStaleStamp) { EXPECT_FALSE(evaluate_fix(valid_fix(), kNow+3000000000LL, {}, {}).accepted); }
TEST(FixQuality, RejectsFutureStamp) { auto f=valid_fix(); f.header.stamp.sec=102; EXPECT_FALSE(evaluate_fix(f,kNow,{},{}).accepted); }
TEST(FixQuality, RejectsBackwardStamp) { EXPECT_FALSE(evaluate_fix(valid_fix(),kNow,{},QualityState{101000000000LL}).accepted); }
TEST(FixQuality, RejectsNonFiniteCovariance) { auto f=valid_fix(); f.position_covariance[0]=std::numeric_limits<double>::infinity(); EXPECT_FALSE(evaluate_fix(f,kNow,{},{}).accepted); }
TEST(FixQuality, RejectsNegativeCovariance) { auto f=valid_fix(); f.position_covariance[0]=-1; EXPECT_FALSE(evaluate_fix(f,kNow,{},{}).accepted); }
TEST(FixQuality, RejectsUnknownCovarianceWhenRequired) { auto f=valid_fix(); f.position_covariance_type=sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN; EXPECT_FALSE(evaluate_fix(f,kNow,{},{}).accepted); }
TEST(FixQuality, RejectsHorizontalThresholdExceeded) { auto f=valid_fix(); f.position_covariance[0]=100; f.position_covariance[4]=100; EXPECT_FALSE(evaluate_fix(f,kNow,{},{}).accepted); }
TEST(FixQuality, RejectsVerticalThresholdExceeded) { auto f=valid_fix(); f.position_covariance[8]=101; EXPECT_FALSE(evaluate_fix(f,kNow,{},{}).accepted); }
TEST(FixQuality, RecoversAfterInvalidFix) { auto f=valid_fix(); f.latitude=91; EXPECT_FALSE(evaluate_fix(f,kNow,{},{}).accepted); EXPECT_TRUE(evaluate_fix(valid_fix(),kNow,{},{}).accepted); }
}  // namespace
