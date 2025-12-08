#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "sailbot/msg/nav_state.hpp"

#include "sailbot/sensors/gnss_sensor.hpp"
#include "sailbot/sensors/imu_sensor.hpp"
#include "sailbot/sensors/wind_sensor.hpp"           // NEW: AS5600 wind sensor
#include "sailbot/estimation/nav_state_estimator.hpp"

namespace sailbot {

using std::placeholders::_1;

class NavStateEstimatorNode : public rclcpp::Node {
public:
  NavStateEstimatorNode()
  : Node("nav_state_estimator_node"),
    gnss_sensor_(),
    nav_estimator_(estimation::NavStateEstimator::Params{}){
    // Parameters
    nmea_topic_  = declare_parameter<std::string>("nmea_topic", "/teseo/nmea");
    imu_topic_   = declare_parameter<std::string>("imu_topic", "/imu/data");
    double rate_hz = declare_parameter<double>("rate_hz", 50.0);

    // Wind I2C params (reads AWA directly from AS5600)
    wind_i2c_dev_  = declare_parameter<std::string>("wind.i2c_device", "/dev/i2c-1");
    wind_i2c_addr_ = declare_parameter<int>("wind.i2c_address", 0x36);

    RCLCPP_INFO(get_logger(), "NavStateEstimatorNode listening to:");
    RCLCPP_INFO(get_logger(), "  NMEA: %s", nmea_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  IMU:  %s", imu_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Wind sensor: %s @ 0x%02X",
                wind_i2c_dev_.c_str(), wind_i2c_addr_);

    // Subscriptions
    nmea_sub_ = create_subscription<std_msgs::msg::String>(
      nmea_topic_, 100,
      std::bind(&NavStateEstimatorNode::nmea_callback, this, _1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 50,
      std::bind(&NavStateEstimatorNode::imu_callback, this, _1));

    // Publishers
    nav_raw_pub_   = create_publisher<sailbot::msg::NavState>("/state/nav_raw", 10);
    nav_fused_pub_ = create_publisher<sailbot::msg::NavState>("/state/nav_fused", 10);
    awa_pub_       = create_publisher<std_msgs::msg::Float32>("/state/wind_apparent", 10);

    // Wind sensor init
    try {
      wind_sensor_ = std::make_unique<sensors::WindSensor>(
          wind_i2c_dev_, static_cast<uint8_t>(wind_i2c_addr_));
      RCLCPP_INFO(get_logger(), "AS5600 opened OK");
    } catch (const std::exception& e) {
      RCLCPP_FATAL(get_logger(), "Failed to init AS5600: %s", e.what());
      throw;
    }

    // Timer for stepping estimator + publishing AWA
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
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    double yaw_rad = std::atan2(siny_cosp, cosy_cosp);

    float yaw_deg = static_cast<float>(yaw_rad * 180.0 / M_PI);
    if (yaw_deg < 0.0f) yaw_deg += 360.0f;

    // Yaw rate from angular_velocity.z (rad/s) → deg/s
    float yaw_rate_deg_s = static_cast<float>(msg->angular_velocity.z * 180.0 / M_PI);

    imu_sensor_.set_yaw_deg(yaw_deg);
    imu_sensor_.set_yaw_rate_deg_s(yaw_rate_deg_s);

    double t_sec = now().seconds();
    nav_estimator_.update_imu(yaw_deg, yaw_rate_deg_s, t_sec);
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
    }

    // --- Wind: publish AWA (deg, [-180,180]) each tick ---
    try {
      const float rad = wind_sensor_->read_angle_rad();                 // [0, 2*pi)
      float awa_deg = rad * (180.0f / static_cast<float>(M_PI));        // [0, 360)
      if (awa_deg >= 180.0f) awa_deg -= 360.0f;                         // [-180, 180)
      std_msgs::msg::Float32 msg;
      msg.data = awa_deg;
      awa_pub_->publish(msg);
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "AS5600 read failed: %s", e.what());
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

  // Sensors and estimators
  sensors::GnssSensor gnss_sensor_;
  sensors::ImuSensor  imu_sensor_;
  estimation::NavStateEstimator nav_estimator_;

  // Wind sensor (AS5600)
  std::unique_ptr<sensors::WindSensor> wind_sensor_;
  std::string wind_i2c_dev_;
  int         wind_i2c_addr_;

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nmea_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

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
