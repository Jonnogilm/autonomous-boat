#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "sailbot/sensors/bno055_i2c.hpp"

class Bno055I2CNode : public rclcpp::Node {
public:
  Bno055I2CNode()
  : Node("bno055_i2c_node")
  {
    // Parameters
    auto dev = declare_parameter<std::string>("i2c_dev", "/dev/i2c-1");
    int addr_int = declare_parameter<int>("i2c_addr_hex", 0x29);
    double rate_hz = declare_parameter<double>("rate_hz", 20.0);
    publish_imu_msg_ = declare_parameter<bool>("publish_imu_msg", true);

    sailbot::sensors::Bno055I2C::Config cfg;
    cfg.i2c_dev = dev;
    cfg.i2c_addr = static_cast<uint8_t>(addr_int & 0xFF);
    cfg.use_ndof = true;

    bno_ = std::make_unique<sailbot::sensors::Bno055I2C>(cfg);

    std::string err;
    if (!bno_->begin(&err)) {
      RCLCPP_FATAL(get_logger(), "BNO055 begin() failed: %s", err.c_str());
      throw std::runtime_error("BNO055 init failed: " + err);
    }

    RCLCPP_INFO(get_logger(), "BNO055 ready on %s addr 0x%02X",
                dev.c_str(), cfg.i2c_addr);

    // Publishers
    heading_pub_ = create_publisher<std_msgs::msg::Float32>("/imu/heading_mag_deg", 10);
    roll_pub_ = create_publisher<std_msgs::msg::Float32>("/imu/roll_deg", 10);
    yaw_rate_pub_ = create_publisher<std_msgs::msg::Float32>("/imu/yaw_rate_deg_s", 10);
    calib_pub_ = create_publisher<std_msgs::msg::UInt8>("/imu/calib_stat", 10);

    if (publish_imu_msg_) {
      imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    }

    // Timer
    auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Bno055I2CNode::tick, this)
    );
  }

private:
  void tick() {
    std::string err;

    auto h = bno_->read_heading_deg(&err);
    if (!h.has_value()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "read_heading_deg failed: %s", err.c_str());
      return;
    }

    auto wz = bno_->read_yaw_rate_deg_s(&err);
    if (!wz.has_value()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "read_yaw_rate_deg_s failed: %s", err.c_str());
      return;
    }

    auto roll = bno_->read_roll_deg(&err);
    if (!roll.has_value()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "read_roll_deg failed: %s", err.c_str());
      // Continue? Roll failure might be acceptable if heading is ok, but typically all I2C reads fail together.
      // We'll proceed.
    }

    auto calib = bno_->read_calib_stat(&err);
    if (!calib.has_value()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "read_calib_stat failed: %s", err.c_str());
      // Continue; heading is still useful, but you should watch this.
    }

    // Publish heading
    std_msgs::msg::Float32 hmsg;
    hmsg.data = *h;
    heading_pub_->publish(hmsg);

    // Publish yaw rate
    std_msgs::msg::Float32 rmsg;
    rmsg.data = *wz;
    yaw_rate_pub_->publish(rmsg);

    // Publish roll
    if (roll.has_value()) {
      std_msgs::msg::Float32 roll_msg;
      roll_msg.data = *roll;
      roll_pub_->publish(roll_msg);
    }

    // Publish calib (if available)
    if (calib.has_value()) {
      std_msgs::msg::UInt8 cmsg;
      cmsg.data = *calib;
      calib_pub_->publish(cmsg);
    }

    // Optional /imu/data publish (quaternion + yaw rate)
    if (publish_imu_msg_ && imu_pub_) {
      sensor_msgs::msg::Imu imsg;
      imsg.header.stamp = now();
      imsg.header.frame_id = "imu_link";

      auto q = bno_->read_quaternion(&err);
      if (q.has_value()) {
        imsg.orientation.w = q->w;
        imsg.orientation.x = q->x;
        imsg.orientation.y = q->y;
        imsg.orientation.z = q->z;
      } else {
        // Signal orientation invalid (ROS convention)
        imsg.orientation_covariance[0] = -1.0;
      }

      // BNO055 gyro output already in deg/s -> convert to rad/s for sensor_msgs/Imu
      constexpr double deg2rad = 3.14159265358979323846 / 180.0;
      imsg.angular_velocity.z = static_cast<double>(*wz) * deg2rad;

      // We are not publishing accel here (not needed for heading hold).
      imsg.linear_acceleration_covariance[0] = -1.0;

      imu_pub_->publish(imsg);
    }
  }

  bool publish_imu_msg_{true};

  std::unique_ptr<sailbot::sensors::Bno055I2C> bno_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr heading_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr roll_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_rate_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr calib_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Bno055I2CNode>());
  rclcpp::shutdown();
  return 0;
}