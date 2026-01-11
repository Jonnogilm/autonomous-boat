#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"

#include "sailbot/sensors/wind_sensor.hpp"

namespace sailbot {

class WindSensorNode : public rclcpp::Node {
public:
  WindSensorNode()
  : Node("wind_sensor_node") {
    // Parameters
    sensors::WindSensor::Params p;
    p.gpiochip = declare_parameter<int>("gpiochip", 4);
    p.gpio_clk = declare_parameter<int>("gpio_clk", 16);
    p.gpio_dt  = declare_parameter<int>("gpio_dt", 20);
    p.gpio_sw  = declare_parameter<int>("gpio_sw", 21);
    p.detents_per_rev = declare_parameter<int>("detents_per_rev", 20);
    p.invert = declare_parameter<bool>("invert", false);
    p.debounce_sec = declare_parameter<double>("debounce_sec", 0.005);

    rate_hz_ = declare_parameter<double>("rate_hz", 20.0);

    // Publishers
    wind_pub_ = create_publisher<std_msgs::msg::Float32>("/wind/raw_angle_deg", 10);

    // If you want to keep older consumers:
    compat_pub_ = create_publisher<std_msgs::msg::Float32>("/state/wind_apparent", 10);

    // Armed publisher should be latched (transient local)
    rclcpp::QoS armed_qos(1);
    armed_qos.transient_local();
    armed_qos.reliable();
    armed_pub_ = create_publisher<std_msgs::msg::Bool>("/system/armed", armed_qos);

    wind_ = std::make_unique<sensors::WindSensor>(p);
    wind_->start();

    auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&WindSensorNode::tick, this));

    RCLCPP_INFO(get_logger(), "WindSensorNode running. Button press will zero angle and publish /system/armed=true.");
  }

  ~WindSensorNode() override {
    if (wind_) wind_->stop();
  }

private:
  void tick() {
    // Publish wind continuously (debug/visibility)
    std_msgs::msg::Float32 w;
    w.data = wind_->angle_deg();
    wind_pub_->publish(w);
    compat_pub_->publish(w);

    // On each button press: publish /system/armed=true again
    if (wind_->consume_button_pressed()) {
      std_msgs::msg::Bool a;
      a.data = true;
      armed_pub_->publish(a);
      RCLCPP_INFO(get_logger(), "ARMED: wind zeroed and /system/armed published.");
    }
  }

  double rate_hz_{20.0};
  std::unique_ptr<sailbot::sensors::WindSensor> wind_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wind_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr compat_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sailbot

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sailbot::WindSensorNode>());
  rclcpp::shutdown();
  return 0;
}
