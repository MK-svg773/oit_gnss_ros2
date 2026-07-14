#include "oit_gnss_monitor/fix_quality.hpp"

#include <cmath>

namespace oit_gnss_monitor {
int64_t stamp_nanoseconds(const builtin_interfaces::msg::Time & stamp) {
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

double horizontal_combined_stddev(const sensor_msgs::msg::NavSatFix & fix) {
  return std::sqrt(fix.position_covariance[0] + fix.position_covariance[4]);
}

double vertical_stddev(const sensor_msgs::msg::NavSatFix & fix) {
  return std::sqrt(fix.position_covariance[8]);
}

QualityResult evaluate_fix(const sensor_msgs::msg::NavSatFix & fix, int64_t now_ns,
                           const QualityLimits & limits, const QualityState & state) {
  std::vector<std::string> reasons;
  const int64_t stamp_ns = stamp_nanoseconds(fix.header.stamp);
  if (!std::isfinite(fix.latitude) || fix.latitude < -90.0 || fix.latitude > 90.0) reasons.emplace_back("invalid latitude");
  if (!std::isfinite(fix.longitude) || fix.longitude < -180.0 || fix.longitude > 180.0) reasons.emplace_back("invalid longitude");
  if (limits.require_finite_altitude && !std::isfinite(fix.altitude)) reasons.emplace_back("invalid altitude");
  if (fix.status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) reasons.emplace_back("no fix");
  if (limits.require_known_covariance && fix.position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) reasons.emplace_back("unknown covariance");
  for (double covariance : fix.position_covariance) {
    if (!std::isfinite(covariance)) { reasons.emplace_back("non-finite covariance"); break; }
    if (covariance < 0.0) { reasons.emplace_back("negative covariance"); break; }
  }
  if (stamp_ns <= 0) reasons.emplace_back("zero timestamp");
  else if (stamp_ns > now_ns + static_cast<int64_t>(limits.max_future_sec * 1e9)) reasons.emplace_back("future timestamp");
  else if (now_ns - stamp_ns > static_cast<int64_t>(limits.max_age_sec * 1e9)) reasons.emplace_back("stale timestamp");
  if (state.last_stamp_ns && stamp_ns < *state.last_stamp_ns) reasons.emplace_back("timestamp moved backwards");
  if (reasons.empty() && horizontal_combined_stddev(fix) > limits.max_horizontal_combined_stddev) reasons.emplace_back("horizontal combined stddev above threshold");
  if (reasons.empty() && vertical_stddev(fix) > limits.max_vertical_stddev) reasons.emplace_back("vertical stddev above threshold");
  return {reasons.empty(), std::move(reasons)};
}
}  // namespace oit_gnss_monitor
