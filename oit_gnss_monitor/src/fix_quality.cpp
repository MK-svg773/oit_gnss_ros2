#include "oit_gnss_monitor/fix_quality.hpp"

#include <cmath>

namespace oit_gnss_monitor {
namespace {
int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp) {
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}
}  // namespace

QualityResult evaluate_fix(const sensor_msgs::msg::NavSatFix & fix, const int64_t now_ns,
                           const QualityLimits & limits, const QualityState & state) {
  std::vector<std::string> reasons;
  const auto message_stamp = stamp_ns(fix.header.stamp);
  if (!std::isfinite(fix.latitude) || fix.latitude < -90.0 || fix.latitude > 90.0) reasons.emplace_back("invalid latitude");
  if (!std::isfinite(fix.longitude) || fix.longitude < -180.0 || fix.longitude > 180.0) reasons.emplace_back("invalid longitude");
  if (fix.status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) reasons.emplace_back("no fix");
  for (const double covariance : fix.position_covariance) {
    if (!std::isfinite(covariance)) { reasons.emplace_back("non-finite covariance"); break; }
  }
  if (message_stamp <= 0 || now_ns - message_stamp > static_cast<int64_t>(limits.max_age_sec * 1e9)) reasons.emplace_back("stale fix");
  if (state.last_stamp_ns && message_stamp < *state.last_stamp_ns) reasons.emplace_back("timestamp moved backwards");
  const double horizontal = std::sqrt(std::max(0.0, fix.position_covariance[0]) + std::max(0.0, fix.position_covariance[4]));
  const double vertical = std::sqrt(std::max(0.0, fix.position_covariance[8]));
  if (horizontal > limits.max_horizontal_stddev) reasons.emplace_back("horizontal stddev above threshold");
  if (vertical > limits.max_vertical_stddev) reasons.emplace_back("vertical stddev above threshold");
  return {reasons.empty(), std::move(reasons)};
}
}  // namespace oit_gnss_monitor
