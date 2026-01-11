#include <chrono>
#include <cmath>
#include <memory>
#include <algorithm>  // for std::max/std::min

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/bool.hpp"   // ADDED

#include "sailbot/control/sheet_controller.hpp"

namespace {

// Normalize to [0,360)
static inline float wrap360(float deg) {
  while (deg >= 360.0f) deg -= 360.0f;
  while (deg < 0.0f) deg += 360.0f;
  return deg;
}

// Shortest signed angle error desired - current in [-180, +180]
static inline float shortest_error_deg(float desired_deg, float current_deg) {
  float e = wrap360(desired_deg) - wrap360(current_deg);
  while (e > 180.0f) e -= 360.0f;
  while (e < -180.0f) e += 360.0f;
  return e;
}

static inline float clampf(float v, float lo, float hi) {
  return std::max(lo, std::min(v, hi));
}

}  // namespace

namespace sailbot {

using std::placeholders::_1;

class ControlNode : public rclcpp::Node {
public:
  ControlNode()
  : Node("control_node"),
    sheet_ctrl_(sailbot::control::SheetController::Params{})
  {
    // --- Parameters (rudder control) ---
    kp_far_         = declare_parameter<double>("rudder.kp_far", 2.0);
    kp_near_        = declare_parameter<double>("rudder.kp_near", 0.6);
    kd_             = declare_parameter<double>("rudder.kd", 0.2);
    max_rudder_deg_ = declare_parameter<double>("rudder.max_deg", 35.0);

    // Gain scheduling: blend from far->near when |error| is below this band.
    near_band_deg_  = declare_parameter<double>("rudder.near_band_deg", 15.0);

    // --- IMU safety / fusion parameters ---
    imu_timeout_sec_      = declare_parameter<double>("imu.timeout_sec", 0.5);
    heading_jump_deg_     = declare_parameter<double>("imu.heading_jump_deg", 30.0);
    jump_rate_margin_deg_ = declare_parameter<double>("imu.jump_rate_margin_deg", 90.0);

    // Weights (your requirement)
    w_mag_good_ = declare_parameter<double>("imu.w_mag_when_cal_good", 0.8);
    w_mag_bad_  = declare_parameter<double>("imu.w_mag_when_cal_bad", 0.2);

    // --- Topics ---
    desired_heading_topic_ = declare_parameter<std::string>("topics.desired_heading", "/guidance/desired_heading_deg");
    heading_mag_topic_     = declare_parameter<std::string>("topics.heading_mag", "/imu/heading_mag_deg");
    yaw_rate_topic_        = declare_parameter<std::string>("topics.yaw_rate", "/imu/yaw_rate_deg_s");
    calib_topic_           = declare_parameter<std::string>("topics.calib", "/imu/calib_stat");
    awa_topic_             = declare_parameter<std::string>("topics.awa", "/state/wind_apparent");

    // ADDED: arm topic
    arm_topic_             = declare_parameter<std::string>("topics.armed", "/system/armed");

    // --- Subscribers ---
    desired_heading_sub_ = create_subscription<std_msgs::msg::Float32>(
      desired_heading_topic_, 10,
      std::bind(&ControlNode::desired_heading_callback, this, _1));

    heading_mag_sub_ = create_subscription<std_msgs::msg::Float32>(
      heading_mag_topic_, 20,
      std::bind(&ControlNode::heading_mag_callback, this, _1));

    yaw_rate_sub_ = create_subscription<std_msgs::msg::Float32>(
      yaw_rate_topic_, 50,
      std::bind(&ControlNode::yaw_rate_callback, this, _1));

    calib_sub_ = create_subscription<std_msgs::msg::UInt8>(
      calib_topic_, 10,
      std::bind(&ControlNode::calib_callback, this, _1));

    awa_sub_ = create_subscription<std_msgs::msg::Float32>(
      awa_topic_, 20,
      std::bind(&ControlNode::awa_callback, this, _1));

    // ADDED: Armed subscriber (latched QoS)
    {
      rclcpp::QoS qos(1);
      qos.reliable();
      qos.transient_local();
      armed_sub_ = create_subscription<std_msgs::msg::Bool>(
        arm_topic_, qos,
        std::bind(&ControlNode::armed_callback, this, _1));
    }

    // --- Publishers ---
    rudder_pub_ = create_publisher<std_msgs::msg::Float32>("/actuators/rudder_cmd_deg", 10);
    sheet_pub_  = create_publisher<std_msgs::msg::Float32>("/actuators/sheet_cmd_pct", 10);

    // Control loop at 50 Hz
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&ControlNode::update, this));

    RCLCPP_INFO(get_logger(), "ControlNode started (IMU-only heading hold). Waiting for /system/armed.");
  }

private:
  // ---------------- Callbacks ----------------

  // ADDED: Armed callback
  void armed_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data) {
      return;
    }

    // Your requirement: re-arming does NOT require a special sequence.
    // We simply enable control and clear freeze.
    armed_ = true;
    frozen_ = false;
    freeze_reason_ = "none";

    // Your requirement (B): If mission hasn't published a desired heading yet,
    // default desired heading to current heading_mag immediately upon arming.
    if (!have_desired_heading_ && have_heading_mag_) {
      desired_heading_deg_ = heading_mag_deg_;
      have_desired_heading_ = true;
      RCLCPP_INFO(get_logger(), "Armed: defaulting desired heading to current heading (%.2f deg).",
                  desired_heading_deg_);
    } else {
      RCLCPP_INFO(get_logger(), "Armed: control enabled.");
    }
  }

  void desired_heading_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    desired_heading_deg_ = wrap360(msg->data);
    have_desired_heading_ = true;
  }

  void heading_mag_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    heading_mag_deg_ = wrap360(msg->data);
    have_heading_mag_ = true;
    last_imu_rx_time_ = now();
  }

  void yaw_rate_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    yaw_rate_deg_s_ = msg->data;
    have_yaw_rate_ = true;
    last_imu_rx_time_ = now();
  }

  void calib_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
    calib_stat_ = msg->data;
    have_calib_ = true;
  }

  void awa_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    awa_deg_ = msg->data; // assuming /state/wind_apparent is degrees
    have_awa_ = true;
  }

  // -------------- Main control loop --------------

  void update() {
    // ADDED: Hard gate on armed
    if (!armed_) {
      return;
    }

    const auto t_now = now();

    // Require desired heading and at least heading + yaw rate streams.
    if (!have_desired_heading_ || !have_heading_mag_ || !have_yaw_rate_) {
      return;
    }

    // IMU timeout => freeze outputs
    if ((t_now - last_imu_rx_time_).seconds() > imu_timeout_sec_) {
      freeze_reason_ = "IMU timeout";
      frozen_ = true;
    }

    // Initialize gyro-integrated heading on first run
    if (!have_heading_imu_) {
      heading_imu_deg_ = heading_mag_deg_;
      have_heading_imu_ = true;
      last_update_time_ = t_now;
    }

    // dt
    double dt = (t_now - last_update_time_).seconds();
    if (dt <= 0.0) dt = 0.02;
    last_update_time_ = t_now;

    // Integrate gyro heading (IMU heading proxy)
    heading_imu_deg_ = wrap360(heading_imu_deg_ + static_cast<float>(yaw_rate_deg_s_ * dt));

    // Determine mag calibration quality
    // calib_stat bits: [1:0] MAG, [5:4] GYR, etc.
    int mag_cal = -1;
    if (have_calib_) {
      mag_cal = (calib_stat_ & 0x03);
    }

    const bool mag_good = (mag_cal >= 3);

    // Fuse heading (your requested weighting behavior)
    const double w_mag = mag_good ? w_mag_good_ : w_mag_bad_;
    const double w_imu = 1.0 - w_mag;
    (void)w_imu; // not used explicitly; kept for clarity

    // Fuse with wrap-aware blending:
    // take error from imu->mag and correct imu toward mag proportionally.
    float e_imu_to_mag = shortest_error_deg(heading_mag_deg_, heading_imu_deg_);
    float fused_heading = wrap360(static_cast<float>(heading_imu_deg_ + w_mag * e_imu_to_mag));

    // Jump detection: if mag jumps too fast relative to yaw rate, freeze.
    {
      float mag_step = std::fabs(shortest_error_deg(heading_mag_deg_, last_heading_mag_deg_));
      // expected max change based on yaw rate plus margin:
      float expected = static_cast<float>(std::fabs(yaw_rate_deg_s_) * dt + jump_rate_margin_deg_ * dt);
      if (mag_step > heading_jump_deg_ && mag_step > expected) {
        frozen_ = true;
        freeze_reason_ = "Heading jump detected";
      }
      last_heading_mag_deg_ = heading_mag_deg_;
    }

    // If frozen, publish last commands (hold servos) and return.
    if (frozen_) {
      publish_hold_commands();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Frozen: %s (holding rudder=%.2f deg, sheet=%.2f pct)",
                           freeze_reason_.c_str(), last_rudder_cmd_deg_, last_sheet_cmd_pct_);
      return;
    }

    // ---------------- Rudder (heading hold) ----------------
    float err_deg = shortest_error_deg(desired_heading_deg_, fused_heading);

    // Gain scheduling: blend kp from far to near when |err| < near_band_deg
    double a = std::fabs(err_deg);
    double blend = 1.0;
    if (near_band_deg_ > 1e-3) {
      blend = clampf(static_cast<float>(a / near_band_deg_), 0.0f, 1.0f); // 0 near, 1 far
    }
    double kp_eff = kp_near_ + (kp_far_ - kp_near_) * blend;

    // PD control (yaw rate is already deg/s)
    float rudder_cmd = static_cast<float>(kp_eff * err_deg - kd_ * yaw_rate_deg_s_);
    rudder_cmd = clampf(rudder_cmd, static_cast<float>(-max_rudder_deg_), static_cast<float>(max_rudder_deg_));

    // Publish rudder
    last_rudder_cmd_deg_ = rudder_cmd;
    std_msgs::msg::Float32 rudder_msg;
    rudder_msg.data = rudder_cmd;
    rudder_pub_->publish(rudder_msg);

    // ---------------- Sheet (from AWA) ----------------
    float sheet_pct = last_sheet_cmd_pct_;

    if (have_awa_) {
      // SheetController expects AWA in degrees (-180..180).
      // If your AWA is 0..360, map to signed.
      float awa_signed = awa_deg_;
      while (awa_signed > 180.0f) awa_signed -= 360.0f;
      while (awa_signed < -180.0f) awa_signed += 360.0f;

      sheet_pct = sheet_ctrl_.compute_from_awa(awa_signed);
    }

    last_sheet_cmd_pct_ = sheet_pct;
    std_msgs::msg::Float32 sheet_msg;
    sheet_msg.data = sheet_pct;
    sheet_pub_->publish(sheet_msg);
  }

  void publish_hold_commands() {
    std_msgs::msg::Float32 rudder_msg;
    rudder_msg.data = last_rudder_cmd_deg_;
    rudder_pub_->publish(rudder_msg);

    std_msgs::msg::Float32 sheet_msg;
    sheet_msg.data = last_sheet_cmd_pct_;
    sheet_pub_->publish(sheet_msg);
  }

  // ---------------- Members ----------------

  // Topics
  std::string desired_heading_topic_;
  std::string heading_mag_topic_;
  std::string yaw_rate_topic_;
  std::string calib_topic_;
  std::string awa_topic_;

  // ADDED
  std::string arm_topic_;

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr desired_heading_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr heading_mag_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_rate_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr calib_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr awa_sub_;

  // ADDED
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr armed_sub_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rudder_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sheet_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // ADDED: armed state
  bool armed_{false};

  // Desired heading
  float desired_heading_deg_{0.0f};
  bool have_desired_heading_{false};

  // IMU inputs
  float heading_mag_deg_{0.0f};
  float yaw_rate_deg_s_{0.0f};
  bool have_heading_mag_{false};
  bool have_yaw_rate_{false};

  uint8_t calib_stat_{0};
  bool have_calib_{false};

  // Integrated heading
  float heading_imu_deg_{0.0f};
  bool have_heading_imu_{false};

  rclcpp::Time last_imu_rx_time_{0,0,RCL_ROS_TIME};
  rclcpp::Time last_update_time_{0,0,RCL_ROS_TIME};

  float last_heading_mag_deg_{0.0f};

  // Wind
  float awa_deg_{0.0f};
  bool have_awa_{false};

  // Sheet controller
  sailbot::control::SheetController sheet_ctrl_;

  // Rudder params
  double kp_far_{2.0};
  double kp_near_{0.6};
  double kd_{0.2};
  double max_rudder_deg_{35.0};
  double near_band_deg_{15.0};

  // Safety/fusion params
  double imu_timeout_sec_{0.5};
  double heading_jump_deg_{30.0};
  double jump_rate_margin_deg_{90.0};

  double w_mag_good_{0.8};
  double w_mag_bad_{0.2};

  // Freeze state
  bool frozen_{false};
  std::string freeze_reason_{"none"};
  float last_rudder_cmd_deg_{0.0f};
  float last_sheet_cmd_pct_{50.0f};
};

}  // namespace sailbot

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sailbot::ControlNode>());
  rclcpp::shutdown();
  return 0;
}
