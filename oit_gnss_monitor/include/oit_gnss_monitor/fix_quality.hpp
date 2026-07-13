#pragma once

#include <optional>
#include <string>
#include <vector>

#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace oit_gnss_monitor {

struct QualityLimits {
  double max_horizontal_stddev{5.0};
  double max_vertical_stddev{10.0};
  double max_age_sec{2.0};
};

struct QualityState {
  std::optional<int64_t> last_stamp_ns;
};

struct QualityResult {
  bool accepted;
  std::vector<std::string> reasons;
};

QualityResult evaluate_fix(const sensor_msgs::msg::NavSatFix & fix, int64_t now_ns,
                           const QualityLimits & limits, const QualityState & state);

}  // namespace oit_gnss_monitor
