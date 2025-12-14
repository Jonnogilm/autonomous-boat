#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>

#include "sailbot/actuators/pca9685.hpp"
#include "sailbot/actuators/rudder_servo.hpp"
#include "sailbot/actuators/winch_servo.hpp"

using namespace std::chrono_literals;

class ActuatorsNode : public rclcpp::Node {
public:
  ActuatorsNode() : Node("actuators_node") {

    // =============================
    // Parameters (I2C / PCA9685)
    declare_parameter<std::string>("i2c_dev", "/dev/i2c-1");
    declare_parameter<int>("i2c_addr_hex", 0x40);
    declare_parameter<double>("oscillator_hz", 25'000'000.0);

    // ONE universal PCA frequency
    declare_parameter<double>("pca_freq_hz", 50.0);

    // Channels
    declare_parameter<int>("rudder_channel", 0);
    declare_parameter<int>("winch_channel", 1);

    // Rudder parameters
    declare_parameter<double>("rudder_min_us", 1000.0);
    declare_parameter<double>("rudder_center_us", 1500.0);
    declare_parameter<double>("rudder_max_us", 2000.0);
    declare_parameter<double>("rudder_min_deg", -35.0);
    declare_parameter<double>("rudder_max_deg",  35.0);
    declare_parameter<double>("rudder_rate_deg_s", 180.0);

    // Winch parameters
    declare_parameter<double>("winch_min_us", 500.0);
    declare_parameter<double>("winch_neutral_us", 1500.0);
    declare_parameter<double>("winch_max_us", 2500.0);
    declare_parameter<double>("winch_deadband_us", 4.0);
    declare_parameter<double>("winch_rate_pct_s", 300.0);

    // =============================
    // Load parameters
    const auto dev  = get_parameter("i2c_dev").as_string();
    const auto addr = static_cast<uint8_t>(get_parameter("i2c_addr_hex").as_int());
    const auto osc  = static_cast<float>(get_parameter("oscillator_hz").as_double());
    pca_freq_hz_    = get_parameter("pca_freq_hz").as_double();

    pca_ = std::make_unique<PCA9685>(dev, addr, osc);
    if (!pca_->begin()) {
      RCLCPP_FATAL(get_logger(), "Failed to initialize PCA9685");
      throw std::runtime_error("PCA9685 init failed");
    }

    // Set PCA frequency ONCE
    pca_->set_pwm_freq(pca_freq_hz_);

    // =============================
    // Construct servos
    rudder_ = std::make_unique<RudderServo>(
      *pca_,
      static_cast<uint8_t>(get_parameter("rudder_channel").as_int()),
      get_parameter("rudder_min_us").as_double(),
      get_parameter("rudder_max_us").as_double(),
      get_parameter("rudder_center_us").as_double(),
      get_parameter("rudder_min_deg").as_double(),
      get_parameter("rudder_max_deg").as_double(),
      get_parameter("rudder_rate_deg_s").as_double()
    );

    winch_ = std::make_unique<WinchServo>(
      *pca_,
      static_cast<uint8_t>(get_parameter("winch_channel").as_int()),
      get_parameter("winch_min_us").as_double(),
      get_parameter("winch_max_us").as_double(),
      get_parameter("winch_neutral_us").as_double(),
      get_parameter("winch_deadband_us").as_double(),
      get_parameter("winch_rate_pct_s").as_double()
    );

    // Deterministic safe startup
    rudder_->set_target_deg(0.0f);
    winch_->set_target_pct(0.0f);

    // =============================
    // Subscriptions (UNCHANGED)
    rudder_sub_legacy_ = create_subscription<std_msgs::msg::Float32>(
      "/actuators/rudder/angle_deg", 10,
      [this](std_msgs::msg::Float32::SharedPtr msg){
        rudder_->set_target_deg(msg->data);
      });

    winch_sub_legacy_ = create_subscription<std_msgs::msg::Float32>(
      "/actuators/winch/pct", 10,
      [this](std_msgs::msg::Float32::SharedPtr msg){
        winch_->set_target_pct(msg->data);
      });

    rudder_sub_ctrl_ = create_subscription<std_msgs::msg::Float32>(
      "/actuators/rudder_cmd_deg", 10,
      [this](std_msgs::msg::Float32::SharedPtr msg){
        rudder_->set_target_deg(msg->data);
      });

    winch_sub_ctrl_ = create_subscription<std_msgs::msg::Float32>(
      "/actuators/sheet_cmd_pct", 10,
      [this](std_msgs::msg::Float32::SharedPtr msg){
        winch_->set_target_pct(msg->data);
      });

    // =============================
    // Control loop (50 Hz)
    timer_ = create_wall_timer(20ms, [this](){
      constexpr double dt = 0.02;
      rudder_->update(dt);
      winch_->update(dt);
    });

    RCLCPP_INFO(get_logger(),
      "Actuators node running (PCA9685 @ %.1f Hz)", pca_freq_hz_);
  }

private:
  std::unique_ptr<PCA9685> pca_;
  std::unique_ptr<RudderServo> rudder_;
  std::unique_ptr<WinchServo> winch_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rudder_sub_legacy_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr winch_sub_legacy_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rudder_sub_ctrl_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr winch_sub_ctrl_;

  rclcpp::TimerBase::SharedPtr timer_;
  double pca_freq_hz_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActuatorsNode>());
  rclcpp::shutdown();
  return 0;
}
