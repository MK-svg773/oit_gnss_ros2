#include <sstream>
#include <utility>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "oit_gnss_monitor/fix_quality.hpp"

namespace oit_gnss_monitor {
class FixQualityMonitor : public rclcpp::Node {
 public:
  FixQualityMonitor() : Node("fix_quality_monitor"), updater_(this) {
    limits_.max_horizontal_stddev = declare_parameter<double>("max_horizontal_stddev", 5.0);
    limits_.max_vertical_stddev = declare_parameter<double>("max_vertical_stddev", 10.0);
    limits_.max_age_sec = declare_parameter<double>("max_age_sec", 2.0);
    if (limits_.max_horizontal_stddev <= 0 || limits_.max_vertical_stddev <= 0 || limits_.max_age_sec <= 0) throw std::runtime_error("quality limits must be positive");
    publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>("/fix", rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>("/fix_raw", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr fix) { callback(std::move(fix)); });
    updater_.setHardwareID("gnss_fix_quality_monitor");
    updater_.add("GNSS fix quality", this, &FixQualityMonitor::diagnose);
  }
 private:
  void callback(sensor_msgs::msg::NavSatFix::ConstSharedPtr fix) {
    result_ = evaluate_fix(*fix, now().nanoseconds(), limits_, state_);
    state_.last_stamp_ns = static_cast<int64_t>(fix->header.stamp.sec) * 1000000000LL + fix->header.stamp.nanosec;
    if (result_.accepted) publisher_->publish(*fix);
    updater_.force_update();
  }
  void diagnose(diagnostic_updater::DiagnosticStatusWrapper & status) {
    if (result_.accepted) { status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "GNSS fix accepted"); return; }
    std::ostringstream stream; for (size_t i = 0; i < result_.reasons.size(); ++i) { if (i) stream << "; "; stream << result_.reasons[i]; }
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, stream.str());
  }
  QualityLimits limits_; QualityState state_; QualityResult result_{false, {"no fix received"}};
  diagnostic_updater::Updater updater_; rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
};
}  // namespace oit_gnss_monitor
int main(int argc, char ** argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<oit_gnss_monitor::FixQualityMonitor>()); rclcpp::shutdown(); return 0; }
