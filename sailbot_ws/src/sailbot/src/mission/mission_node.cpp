#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "sailbot/msg/nav_state.hpp"
#include "sailbot/msg/guidance_debug.hpp"
#include "sailbot/msg/mission_state.hpp"

#include "sailbot/mission/fsm.hpp"

namespace sailbot {

using std::placeholders::_1;

class MissionNode : public rclcpp::Node {
public:
  MissionNode()
  : Node("mission_node"),
    fsm_params_(),
    fsm_(fsm_params_) {

    // Parameters
    std::string nav_topic = declare_parameter<std::string>(
      "nav_topic", "/state/nav_fused");
    std::string dbg_topic = declare_parameter<std::string>(
      "guidance_debug_topic", "/guidance/debug");

    fsm_params_.min_fix_quality = declare_parameter<int>(
      "mission.min_fix_quality", 1);
    fsm_params_.wp_reached_margin_m =
      declare_parameter<double>("mission.wp_reached_margin_m", 5.0);

    fsm_ = mission::MissionFsm(fsm_params_);

    double rate_hz = declare_parameter<double>(
      "rate_hz", 10.0);  // mission logic doesn't need to be super fast

    // Subscriptions
    nav_sub_ = create_subscription<sailbot::msg::NavState>(
      nav_topic, 10,
      std::bind(&MissionNode::nav_callback, this, _1));

    dbg_sub_ = create_subscription<sailbot::msg::GuidanceDebug>(
      dbg_topic, 10,
      std::bind(&MissionNode::debug_callback, this, _1));

    // Publishers
    mission_pub_ = create_publisher<sailbot::msg::MissionState>(
      "/mission/state", 10);

    sheet_override_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/mission/sheet_override_pct", 10);

    auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MissionNode::timer_step, this));
  }

private:
  void nav_callback(const sailbot::msg::NavState::SharedPtr msg) {
    last_nav_ = *msg;
    has_nav_ = true;
  }

  void debug_callback(const sailbot::msg::GuidanceDebug::SharedPtr msg) {
    last_dbg_ = *msg;
    has_dbg_ = true;
  }

  void timer_step() {
    if (!has_nav_ || !has_dbg_) {
      return;
    }

    mission::MissionFsmInput in;
    in.gnss_valid = last_nav_.gnss_valid;
    in.gnss_fix_quality = last_nav_.gnss_fix_quality;
    in.waypoint_reached = last_dbg_.waypoint_reached;
    in.distance_to_wp_m = last_dbg_.distance_to_wp_m;
    in.t_sec = now().seconds();

    mission::MissionFsmOutput out = fsm_.step(in);

    // Publish MissionState
    sailbot::msg::MissionState msg;
    msg.stamp = now();
    msg.mode = static_cast<uint8_t>(out.mode);
    msg.gnss_ready = out.gnss_ready;
    msg.waypoint_reached = out.waypoint_reached;
    msg.distance_to_wp_m = in.distance_to_wp_m;
    mission_pub_->publish(msg);

    // Publish sheet override: 100% when mission says dump sails, else 0.
    std_msgs::msg::Float32 sheet_msg;
    sheet_msg.data = out.dump_sails ? 100.0f : 0.0f;
    sheet_override_pub_->publish(sheet_msg);
  }

  // FSM
  mission::MissionFsmParams fsm_params_;
  mission::MissionFsm fsm_;

  // State
  sailbot::msg::NavState last_nav_;
  sailbot::msg::GuidanceDebug last_dbg_;
  bool has_nav_ = false;
  bool has_dbg_ = false;

  // ROS interfaces
  rclcpp::Subscription<sailbot::msg::NavState>::SharedPtr nav_sub_;
  rclcpp::Subscription<sailbot::msg::GuidanceDebug>::SharedPtr dbg_sub_;

  rclcpp::Publisher<sailbot::msg::MissionState>::SharedPtr mission_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sheet_override_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sailbot

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sailbot::MissionNode>());
  rclcpp::shutdown();
  return 0;
}
