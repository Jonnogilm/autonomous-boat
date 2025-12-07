#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "sailbot/msg/nav_state.hpp"

#include "sailbot/control/rudder_controller.hpp"
#include "sailbot/control/sheet_controller.hpp"

namespace sailbot {

using std::placeholders::_1;

class ControlNode : public rclcpp::Node {
public:
  ControlNode()
  : Node("control_node"),
  rudder_ctrl_(sailbot::control::RudderController()),
  sheet_ctrl_(sailbot::control::SheetController()){

    // Parameters
    rudder_kp_ = declare_parameter<double>("rudder.kp", 2.0);
    rudder_kd_ = declare_parameter<double>("rudder.kd", 0.2);
    rudder_limit_deg_ = declare_parameter<double>("rudder.max_deg", 35.0);

    // Subscribers
    desired_heading_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/guidance/desired_heading_deg", 10,
      std::bind(&ControlNode::desired_heading_callback, this, _1));

    nav_sub_ = create_subscription<sailbot::msg::NavState>(
      "/state/nav_fused", 10,
      std::bind(&ControlNode::nav_callback, this, _1));

    sheet_override_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/mission/sheet_override_pct", 10,
      std::bind(&ControlNode::sheet_override_callback, this, _1));

    // Publishers
    rudder_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/actuators/rudder_cmd_deg",
      10);

    sheet_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/actuators/sheet_cmd_pct",
      10);

    // 50 Hz timer
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&ControlNode::update, this));
  }

private:
  // --- Subscriber Callbacks ---
  void desired_heading_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    desired_heading_deg_ = msg->data;
    have_desired_heading_ = true;
  }

  void nav_callback(const sailbot::msg::NavState::SharedPtr msg) {
    last_nav_ = *msg;
    have_nav_ = true;
  }

  void sheet_override_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    sheet_override_pct_ = msg->data;
    have_sheet_override_ = true;
  }

  // --- Main Control Loop ---
  void update() {
    if (!have_desired_heading_ || !have_nav_) {
      return;
    }

    float heading = last_nav_.heading_fused_deg;
    float yaw_rate = last_nav_.yaw_rate_deg_s;

    // Compute rudder
    float rudder_cmd = rudder_ctrl_.compute(
      desired_heading_deg_,
      heading,
      yaw_rate,
      rudder_kp_,
      rudder_kd_,
      rudder_limit_deg_);

    // Publish rudder command
    std_msgs::msg::Float32 rudder_msg;
    rudder_msg.data = rudder_cmd;
    rudder_pub_->publish(rudder_msg);

    // Compute sheet command
    float sheet_pct = 0.0f;

    if (have_sheet_override_ && sheet_override_pct_ > 0.0f) {
      sheet_pct = sheet_override_pct_;  // mission overrides trim logic
    } else {
      sheet_pct = sheet_ctrl_.compute_sheet_pct(last_nav_);
    }

    std_msgs::msg::Float32 sheet_msg;
    sheet_msg.data = sheet_pct;
    sheet_pub_->publish(sheet_msg);
  }

  // --- Data Members ---
  sailbot::msg::NavState last_nav_;
  bool have_nav_ = false;

  float desired_heading_deg_ = 0.0f;
  bool have_desired_heading_ = false;

  float sheet_override_pct_ = 0.0f;
  bool have_sheet_override_ = false;

  // Controllers
  sailbot::control::RudderController rudder_ctrl_;
  sailbot::control::SheetController sheet_ctrl_;
  

  // Parameters
  double rudder_kp_, rudder_kd_, rudder_limit_deg_;

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr desired_heading_sub_;
  rclcpp::Subscription<sailbot::msg::NavState>::SharedPtr nav_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sheet_override_sub_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rudder_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sheet_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace sailbot

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sailbot::ControlNode>());
  rclcpp::shutdown();
  return 0;
}