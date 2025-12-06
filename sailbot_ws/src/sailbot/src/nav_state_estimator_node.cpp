#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "sailbot/msg/nav_state.hpp"

#include "sailbot/sensors/gnss_sensor.hpp"
#include "sailbot/sensors/imu_sensor.hpp"
#include "sailbot/estimation/nav_state_estimator.hpp"
#include "sailbot/estimation/true_wind_estimator.hpp"

namespace sailbot {

using std::placeholders::_1;

class NavStateEstimatorNode : public rclcpp::Node {
public:
  NavStateEstimatorNode()
  : Node("nav_state_estimator_node"),
    gnss_sensor_(),
    nav_estimator_(estimation::NavStateEstimator::Params{}),
    true_wind_() {

    // Parameters
    nmea_topic_  = declare_parameter<std::string>("nmea_topic", "/teseo/nmea");
    imu_topic_   = declare_parameter<std::string>("imu_topic", "/imu/data");
    wind_topic_  = declare_parameter<std::string>("wind_topic", "/sensors/wind/angle_rad");
    double rate_hz = declare_parameter<double>("rate_hz", 50.0);

    RCLCPP_INFO(get_logger(), "NavStateEstimatorNode listening to:");
    RCLCPP_INFO(get_logger(), "  NMEA: %s", nmea_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  IMU:  %s", imu_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  Wind: %s", wind_topic_.c_str());

    // Subscriptions
    nmea_sub_ = create_subscription<std_msgs::msg::String>(
      nmea_topic_, 100,
      std::bind(&NavStateEstimatorNode::nmea_callback, this, _1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 50,
      std::bind(&NavStateEstimatorNode::imu_callback, this, _1));

    wind_sub_ = create_subscription<std_msgs::msg::Float32>(
      wind_topic_, 50,
      std::bind(&NavStateEstimatorNode::wind_callback, this, _1));

    // Publishers
    nav_raw_pub_ = create_publisher<sailbot::msg::NavState>("/state/nav_raw", 10);
    nav_fused_pub_ = create_publisher<sailbot::msg::NavState>("/state/nav_fused", 10);
    awa_pub_ = create_publisher<std_msgs::msg::Float32>("/state/wind_apparent", 10);

    // Timer for stepping estimator
    auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&NavStateEstimatorNode::timer_step, this));
  }

private:
  // --- Callbacks ---

  void nmea_callback(const std_msgs::msg::String::SharedPtr msg) {
    const std::string& sentence = msg->data;
    gnss_sensor_.handle_nmea_sentence(sentence);

    auto fix_opt = gnss_sensor_.last_fix();
    if (!fix_opt.has_value())
      return;

    double t_sec = now().seconds();
    nav_estimator_.update_gnss(fix_opt.value(), t_sec);
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // Extract yaw (heading) from quaternion.
    const auto& q = msg->orientation;
    // Standard ENU yaw from quaternion
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    double yaw_rad = std::atan2(siny_cosp, cosy_cosp);

    float yaw_deg = static_cast<float>(yaw_rad * 180.0 / M_PI);
    if (yaw_deg < 0.0f) yaw_deg += 360.0f;

    // Yaw rate from angular_velocity.z (rad/s).
    float yaw_rate_deg_s = static_cast<float>(msg->angular_velocity.z * 180.0 / M_PI);

    imu_sensor_.set_yaw_deg(yaw_deg);
    imu_sensor_.set_yaw_rate_deg_s(yaw_rate_deg_s);

    double t_sec = now().seconds();
    nav_estimator_.update_imu(yaw_deg, yaw_rate_deg_s, t_sec);
  }

  void wind_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    latest_vane_angle_rad_ = msg->data;
    has_latest_vane_ = true;
  }

  void timer_step() {
    double t_sec = now().seconds();
    nav_estimator_.step(t_sec);

    auto raw_opt   = nav_estimator_.raw_state();
    auto fused_opt = nav_estimator_.fused_state();

    // Publish NavState (raw / fused)
    if (raw_opt.has_value()) {
      publish_nav_state(nav_raw_pub_, raw_opt.value());
    }

    if (fused_opt.has_value()) {
      publish_nav_state(nav_fused_pub_, fused_opt.value());

      // Update and publish apparent wind angle if we have vane data.
      if (has_latest_vane_) {
        true_wind_.update(latest_vane_angle_rad_, fused_opt->heading_fused_deg);
        auto awa = true_wind_.apparent();
        if (awa.valid) {
          std_msgs::msg::Float32 msg;
          msg.data = awa.awa_deg;
          awa_pub_->publish(msg);
        }
      }
    }
  }

  void publish_nav_state(
      const rclcpp::Publisher<sailbot::msg::NavState>::SharedPtr& pub,
      const estimation::NavState& st) {

    sailbot::msg::NavState msg;

    msg.stamp = now();

    msg.lat_deg = st.lat_deg;
    msg.lon_deg = st.lon_deg;
    msg.alt_m   = st.alt_m;

    msg.sog_mps   = st.sog_mps;
    msg.sog_knots = st.sog_knots;

    msg.cog_deg            = st.cog_deg;
    msg.heading_imu_deg    = st.heading_imu_deg;
    msg.heading_fused_deg  = st.heading_fused_deg;
    msg.yaw_rate_deg_s     = st.yaw_rate_deg_s;

    msg.gnss_valid       = st.gnss_valid;
    msg.gnss_fix_quality = st.gnss_fix_quality;
    msg.gnss_num_sat     = st.gnss_num_sat;

    pub->publish(msg);
  }

  // --- Members ---

  // Topics
  std::string nmea_topic_;
  std::string imu_topic_;
  std::string wind_topic_;

  // Sensors and estimators
  sensors::GnssSensor gnss_sensor_;
  sensors::ImuSensor  imu_sensor_;
  estimation::NavStateEstimator nav_estimator_;
  estimation::TrueWindEstimator true_wind_;

  // Vane data
  bool  has_latest_vane_ = false;
  float latest_vane_angle_rad_ = 0.0f;

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nmea_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr wind_sub_;

  rclcpp::Publisher<sailbot::msg::NavState>::SharedPtr nav_raw_pub_;
  rclcpp::Publisher<sailbot::msg::NavState>::SharedPtr nav_fused_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr awa_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sailbot

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sailbot::NavStateEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
