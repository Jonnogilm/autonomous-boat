#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "sailbot/msg/nav_state.hpp"
#include "sailbot/msg/guidance_debug.hpp"

#include "sailbot/guidance/waypoint_nav.hpp"
#include "sailbot/guidance/tack_planner.hpp"
#include "sailbot/guidance/wind_model.hpp"

namespace sailbot {

using std::placeholders::_1;

class GuidanceNode : public rclcpp::Node {
public:
  GuidanceNode()
  : Node("guidance_node"),
    wp_params_(),
    tack_params_(),
    wind_params_(),
    wp_nav_(wp_params_),
    tack_planner_(tack_params_),
    wind_model_(wind_params_) {

    // Declare parameters
    std::string nav_topic = declare_parameter<std::string>(
      "nav_topic", "/state/nav_fused");
    std::string awa_topic = declare_parameter<std::string>(
      "awa_topic", "/state/wind_apparent");

    // Waypoint config
    double wp_lat = declare_parameter<double>("waypoint.lat_deg", 0.0);
    double wp_lon = declare_parameter<double>("waypoint.lon_deg", 0.0);

    // Reference origin; if not provided, default to waypoint.
    double ref_lat = declare_parameter<double>(
      "reference_origin.lat_deg", wp_lat);
    double ref_lon = declare_parameter<double>(
      "reference_origin.lon_deg", wp_lon);

    wp_params_.ref_lat_deg = ref_lat;
    wp_params_.ref_lon_deg = ref_lon;
    wp_params_.acceptance_radius_m =
      declare_parameter<double>("waypoint.acceptance_radius_m", 5.0);

    // Re-build ENU reference in navigator
    wp_nav_ = guidance::WaypointNavigator(wp_params_);
    wp_nav_.set_target(wp_lat, wp_lon);

    RCLCPP_INFO(get_logger(),
      "GuidanceNode waypoint: (%f, %f), origin: (%f, %f), radius: %.1f m",
      wp_lat, wp_lon, ref_lat, ref_lon, wp_params_.acceptance_radius_m);

    // Tack planner params
    tack_params_.no_go_angle_deg =
      declare_parameter<double>("tack.no_go_angle_deg", 40.0);
    tack_planner_ = guidance::TackPlanner(tack_params_);

    // Wind model params
    wind_params_.awa_alpha =
      declare_parameter<double>("wind.awa_alpha", 0.2);
    wind_model_ = guidance::WindModel(wind_params_);

    // Subscriptions
    nav_sub_ = create_subscription<sailbot::msg::NavState>(
      nav_topic, 10,
      std::bind(&GuidanceNode::nav_callback, this, _1));

    awa_sub_ = create_subscription<std_msgs::msg::Float32>(
      awa_topic, 10,
      std::bind(&GuidanceNode::awa_callback, this, _1));

    // Publishers
    desired_heading_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/guidance/desired_heading_deg", 10);

    debug_pub_ = create_publisher<sailbot::msg::GuidanceDebug>(
      "/guidance/debug", 10);

    // Timer at 50 Hz
    double rate_hz = declare_parameter<double>("rate_hz", 50.0);
    auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&GuidanceNode::timer_step, this));
  }

private:
  void nav_callback(const sailbot::msg::NavState::SharedPtr msg) {
    last_nav_state_ = *msg;
    has_nav_state_ = true;
  }

  void awa_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    float raw_awa_deg = msg->data;
    wind_model_.update(raw_awa_deg);
  }

  void timer_step() {
    if (!has_nav_state_ || !wind_model_.has_awa()) {
      return;
    }

    const auto& nav = last_nav_state_;
    const float heading = nav.heading_fused_deg;
    const float awa_smoothed = wind_model_.smoothed_awa_deg();

    // LOS to waypoint
    auto res = wp_nav_.compute(nav.lat_deg, nav.lon_deg);

    bool waypoint_reached = res.target_reached;
    float desired_heading_deg = heading; // default: hold heading
    bool tacking = false;

    if (waypoint_reached) {
      // For now: hold current heading.
      // Your mission FSM can separately command sails fully out, etc.
      desired_heading_deg = heading;
    } else {
      // Tack planner chooses direct course vs. close-hauled tack.
      desired_heading_deg = tack_planner_.compute_desired_heading(
        res.bearing_deg, heading, awa_smoothed);

      // Heuristic: if desired heading deviates significantly from bearing, call that "tacking".
      float diff = std::fabs(wrap180(desired_heading_deg - res.bearing_deg));
      tacking = (diff > 5.0f); // arbitrary small threshold
    }

    // Publish desired heading
    std_msgs::msg::Float32 dh_msg;
    dh_msg.data = desired_heading_deg;
    desired_heading_pub_->publish(dh_msg);

    // Publish debug
    sailbot::msg::GuidanceDebug dbg;
    dbg.stamp = now();
    dbg.bearing_to_wp_deg = res.bearing_deg;
    dbg.distance_to_wp_m = res.distance_m;
    dbg.desired_heading_deg = desired_heading_deg;
    dbg.awa_smoothed_deg = awa_smoothed;
    dbg.tacking = tacking;
    dbg.waypoint_reached = waypoint_reached;
    debug_pub_->publish(dbg);
  }

  static float wrap180(float deg) {
    float a = std::fmod(deg, 360.0f);
    if (a < 0.0f) a += 360.0f;
    if (a >= 180.0f) a -= 360.0f;
    return a;
  }

  // Params and helpers
  guidance::WaypointNavParams wp_params_;
  guidance::TackPlannerParams tack_params_;
  guidance::WindModelParams wind_params_;

  guidance::WaypointNavigator wp_nav_;
  guidance::TackPlanner tack_planner_;
  guidance::WindModel wind_model_;

  // State
  sailbot::msg::NavState last_nav_state_;
  bool has_nav_state_ = false;

  // ROS interfaces
  rclcpp::Subscription<sailbot::msg::NavState>::SharedPtr nav_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr awa_sub_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr desired_heading_pub_;
  rclcpp::Publisher<sailbot::msg::GuidanceDebug>::SharedPtr debug_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sailbot

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sailbot::GuidanceNode>());
  rclcpp::shutdown();
  return 0;
}
