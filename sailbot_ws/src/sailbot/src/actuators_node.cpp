#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include "sailbot/actuators/pca9685.hpp"
#include "sailbot/actuators/rudder_servo.hpp"
#include "sailbot/actuators/winch_servo.hpp"

class ActuatorsNode : public rclcpp::Node {
public:
  ActuatorsNode() : Node("actuators_node") {
    // Params
    declare_parameter<std::string>("i2c_dev", "/dev/i2c-1");
    declare_parameter<int>("i2c_addr_hex", 0x40);
    declare_parameter<double>("oscillator_hz", 25'000'000.0);

    // Frequencies
    declare_parameter<double>("rudder_freq_hz", 50.0);
    declare_parameter<double>("winch_freq_hz", 330.0);

    // Channels
    declare_parameter<int>("rudder_channel", 0);
    declare_parameter<int>("winch_channel", 1);

    // Rudder pulses/us + limits
    declare_parameter<double>("rudder_min_us", 1000.0);
    declare_parameter<double>("rudder_center_us", 1500.0);
    declare_parameter<double>("rudder_max_us", 2000.0);
    declare_parameter<double>("rudder_min_deg", -35.0);
    declare_parameter<double>("rudder_max_deg", +35.0);
    declare_parameter<double>("rudder_rate_deg_s", 180.0);

    // Winch pulses/us + deadband
    declare_parameter<double>("winch_min_us", 500.0);
    declare_parameter<double>("winch_neutral_us", 1500.0);
    declare_parameter<double>("winch_max_us", 2500.0);
    declare_parameter<double>("winch_deadband_us", 4.0);
    declare_parameter<double>("winch_rate_pct_s", 300.0);

    // Load params
    auto dev = get_parameter("i2c_dev").as_string();
    auto addr = static_cast<uint8_t>(get_parameter("i2c_addr_hex").as_int());
    auto osc  = static_cast<float>(get_parameter("oscillator_hz").as_double());

    pca_ = std::make_unique<PCA9685>(dev, addr, osc);
    if (!pca_->begin()) {
      RCLCPP_FATAL(get_logger(), "Failed to init PCA9685");
      throw std::runtime_error("PCA9685 init failed");
    }

    // We’ll program frequency per move to ensure each channel is at its intended freq
    // Strategy: set frequency separately before writing each channel (cheap operation).
    rudder_freq_ = get_parameter("rudder_freq_hz").as_double();
    winch_freq_  = get_parameter("winch_freq_hz").as_double();

    // Construct servos
    rudder_ = std::make_unique<RudderServo>(*pca_,
      static_cast<uint8_t>(get_parameter("rudder_channel").as_int()),
      get_parameter("rudder_min_us").as_double(),
      get_parameter("rudder_max_us").as_double(),
      get_parameter("rudder_center_us").as_double(),
      get_parameter("rudder_min_deg").as_double(),
      get_parameter("rudder_max_deg").as_double(),
      get_parameter("rudder_rate_deg_s").as_double());

    winch_ = std::make_unique<WinchServo>(*pca_,
      static_cast<uint8_t>(get_parameter("winch_channel").as_int()),
      get_parameter("winch_min_us").as_double(),
      get_parameter("winch_max_us").as_double(),
      get_parameter("winch_neutral_us").as_double(),
      get_parameter("winch_deadband_us").as_double(),
      get_parameter("winch_rate_pct_s").as_double());

    // Subs
    rudder_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/actuators/rudder/angle_deg", 10,
      [this](std_msgs::msg::Float32::SharedPtr msg){
        rudder_->set_target_deg(msg->data);
      });

    winch_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/actuators/winch/pct", 10,
      [this](std_msgs::msg::Float32::SharedPtr msg){
        winch_->set_target_pct(msg->data);
      });

    // Control loop timer (50 Hz)
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this](){
      const double dt = 0.02;
      // Ensure proper frequency before commanding each class of servo
      if (std::abs(pca_->current_freq_hz() - rudder_freq_) > 0.5) pca_->set_pwm_freq(rudder_freq_);
      rudder_->update(dt);

      if (std::abs(pca_->current_freq_hz() - winch_freq_) > 0.5) pca_->set_pwm_freq(winch_freq_);
      winch_->update(dt);
    });

    RCLCPP_INFO(get_logger(), "Actuators node started.");
  }

private:
    std::unique_ptr<PCA9685> pca_;
    std::unique_ptr<RudderServo> rudder_;
    std::unique_ptr<WinchServo> winch_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rudder_sub_, winch_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double rudder_freq_, winch_freq_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActuatorsNode>());
  rclcpp::shutdown();
  return 0;
}
