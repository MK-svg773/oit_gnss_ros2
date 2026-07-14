#include <chrono>
#include <optional>
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
    limits_.max_horizontal_combined_stddev = declare_parameter<double>("max_horizontal_combined_stddev", 5.0);
    limits_.max_vertical_stddev = declare_parameter<double>("max_vertical_stddev", 10.0);
    limits_.max_age_sec = declare_parameter<double>("max_age_sec", 2.0);
    limits_.require_finite_altitude = declare_parameter<bool>("require_finite_altitude", false);
    limits_.require_known_covariance = declare_parameter<bool>("require_known_covariance", true);
    publish_location_ = declare_parameter<bool>("diagnostics.publish_location", false);
    if (limits_.max_horizontal_combined_stddev <= 0.0 || limits_.max_vertical_stddev <= 0.0 || limits_.max_age_sec <= 0.0) throw std::runtime_error("quality limits must be positive");
    publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>("/fix", rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>("/fix_raw", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::NavSatFix::ConstSharedPtr fix) { callback(fix); });
    updater_.setHardwareID("gnss_fix_quality_monitor");
    updater_.add("GNSS fix quality", this, &FixQualityMonitor::diagnose);
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() { updater_.force_update(); });
  }
 private:
  void callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr & fix) {
    latest_fix_ = *fix;
    last_receive_time_ = now();
    result_ = evaluate_fix(*fix, last_receive_time_->nanoseconds(), limits_, state_);
    state_.last_stamp_ns = stamp_nanoseconds(fix->header.stamp);
    if (result_.accepted) { ++accepted_count_; publisher_->publish(*fix); } else { ++rejected_count_; }
    updater_.force_update();
  }
  void diagnose(diagnostic_updater::DiagnosticStatusWrapper & status) {
    const auto now_time = now();
    if (!last_receive_time_) { status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "no fix received"); return; }
    const double receive_age = (now_time - *last_receive_time_).seconds();
    const double stamp_age = (now_time.nanoseconds() - stamp_nanoseconds(latest_fix_.header.stamp)) / 1e9;
    if (receive_age > limits_.max_age_sec) status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "GNSS fix stream timeout");
    else if (!result_.accepted) status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, reason_text());
    else status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "GNSS fix accepted");
    status.add("last receive age [sec]", receive_age); status.add("last message stamp age [sec]", stamp_age);
    status.add("horizontal combined stddev [m]", horizontal_combined_stddev(latest_fix_)); status.add("vertical stddev [m]", vertical_stddev(latest_fix_));
    status.add("accepted count", accepted_count_); status.add("rejected count", rejected_count_); status.add("rejection reasons", reason_text());
    if (publish_location_) { status.add("latitude", latest_fix_.latitude); status.add("longitude", latest_fix_.longitude); }
  }
  std::string reason_text() const { std::ostringstream output; for (size_t i = 0; i < result_.reasons.size(); ++i) { if (i) output << "; "; output << result_.reasons[i]; } return output.str(); }
  QualityLimits limits_; QualityState state_; QualityResult result_{false, {"no fix received"}}; bool publish_location_{false};
  std::optional<rclcpp::Time> last_receive_time_; sensor_msgs::msg::NavSatFix latest_fix_; uint64_t accepted_count_{0}; uint64_t rejected_count_{0};
  diagnostic_updater::Updater updater_; rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_; rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_; rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace oit_gnss_monitor
int main(int argc, char ** argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<oit_gnss_monitor::FixQualityMonitor>()); rclcpp::shutdown(); return 0; }
