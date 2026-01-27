#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

namespace {

// Normalize to [0,360)
static inline float wrap360(float deg) {
  float a = std::fmod(deg, 360.0f);
  if (a < 0.0f) a += 360.0f;
  return a;
}

// Map 0..360 -> -180..+180
static inline float wrap180(float deg) {
  float a = wrap360(deg);
  if (a >= 180.0f) a -= 360.0f;
  return a;
}

// Shortest signed error desired-current in [-180,+180]
static inline float shortest_error_deg(float desired_deg, float current_deg) {
  float e = wrap360(desired_deg) - wrap360(current_deg);
  while (e > 180.0f) e -= 360.0f;
  while (e < -180.0f) e += 360.0f;
  return e;
}

static inline float signf(float x) { return (x >= 0.0f) ? 1.0f : -1.0f; }

}  // namespace

namespace sailbot {

using std::placeholders::_1;

class MissionNode : public rclcpp::Node {
public:
  MissionNode()
  : Node("mission_node") {
    // ---------------- Parameters ----------------
    rate_hz_ = declare_parameter<double>("rate_hz", 50.0);

    // Topics
    armed_topic_    = declare_parameter<std::string>("topics.armed", "/system/armed");
    heading_topic_  = declare_parameter<std::string>("topics.heading_deg", "/imu/heading_mag_deg");
    yaw_rate_topic_ = declare_parameter<std::string>("topics.yaw_rate_deg_s", "/imu/yaw_rate_deg_s");
    roll_topic_     = declare_parameter<std::string>("topics.roll_deg", "/imu/roll_deg");
    awa_topic_      = declare_parameter<std::string>("topics.awa_deg_0_360", "/state/wind_apparent");

    desired_heading_pub_topic_ = declare_parameter<std::string>(
      "topics.desired_heading_pub", "/guidance/desired_heading_deg");
    sheet_override_pub_topic_ = declare_parameter<std::string>(
      "topics.sheet_override_pub", "/mission/sheet_override_pct");

    // Upwind / tack params
    no_go_deg_          = declare_parameter<double>("sailing.no_go_deg", 40.0);
    tack_period_sec_    = declare_parameter<double>("tack.period_sec", 60.0);
    pretack_offset_deg_ = declare_parameter<double>("tack.pretack_offset_deg", 10.0);

    // Yaw authority gate params
    yaw_rate_pub_     = declare_parameter<double>("tack.yaw_rate_pub", 5.0);
    heading_delta_min_deg_  = declare_parameter<double>("tack.heading_delta_min_deg", 10.0);
    response_window_sec_    = declare_parameter<double>("tack.response_window_sec", 2.0);
    retry_interval_sec_     = declare_parameter<double>("tack.retry_interval_sec", 3.0);

    // Roll dump params
    roll_dump_trip_deg_  = declare_parameter<double>("safety.roll_dump_trip_deg", 40.0);
    roll_dump_clear_deg_ = declare_parameter<double>("safety.roll_dump_clear_deg", 35.0);
    roll_trip_sec_       = declare_parameter<double>("safety.roll_trip_sec", 0.3);
    roll_clear_sec_      = declare_parameter<double>("safety.roll_clear_sec", 1.0);

    // Mode: for now we run “upwind timed tack pattern” whenever armed.
    // If you later add other modes, this becomes a state machine input.

    // ---------------- Subscriptions ----------------
    // Armed: make it robust for late joiners by using transient local if available.
    // NOTE: Not all RMWs fully respect transient_local on std_msgs/Bool; still useful.
    rclcpp::QoS qos_armed(1);
    qos_armed.reliable().transient_local();

    armed_sub_ = create_subscription<std_msgs::msg::Bool>(
      armed_topic_, qos_armed,
      std::bind(&MissionNode::armed_cb, this, _1));

    heading_sub_ = create_subscription<std_msgs::msg::Float32>(
      heading_topic_, 50,
      std::bind(&MissionNode::heading_cb, this, _1));

    yaw_rate_sub_ = create_subscription<std_msgs::msg::Float32>(
      yaw_rate_topic_, 100,
      std::bind(&MissionNode::yaw_rate_cb, this, _1));

    roll_sub_ = create_subscription<std_msgs::msg::Float32>(
      roll_topic_, 50,
      std::bind(&MissionNode::roll_cb, this, _1));

    awa_sub_ = create_subscription<std_msgs::msg::Float32>(
      awa_topic_, 50,
      std::bind(&MissionNode::awa_cb, this, _1));

    // ---------------- Publishers ----------------
    desired_heading_pub_ = create_publisher<std_msgs::msg::Float32>(
      desired_heading_pub_topic_, 10);

    sheet_override_pub_ = create_publisher<std_msgs::msg::Float32>(
      sheet_override_pub_topic_, 10);

    // ---------------- Timer ----------------
    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MissionNode::step, this));

    RCLCPP_INFO(get_logger(), "MissionNode started: upwind timed-tack pattern, yaw-authority gate, roll dump.");
  }

private:
  // ---------------- Callbacks ----------------
  void armed_cb(const std_msgs::msg::Bool::SharedPtr msg) {
    const bool new_armed = msg->data;

    // Rising edge: initialize mission state
    if (new_armed && !armed_) {
      // Initialize tack timing and pick initial tack based on current AWA sign if available.
      last_tack_commit_time_ = now();
      last_retry_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      state_ = State::RUN;

      if (have_awa_) {
        const float awa_signed = wrap180(awa_deg_0_360_);
        tack_side_ = (awa_signed >= 0.0f) ? TackSide::STARBOARD : TackSide::PORT;
      } else {
        tack_side_ = TackSide::STARBOARD;  // default
      }

      // Reset pretack gate
      pretack_active_ = false;
      pretack_start_time_ = rclcpp::Time(0,0,get_clock()->get_clock_type());
      pretack_start_heading_ = heading_deg_;

      RCLCPP_INFO(get_logger(), "ARMED: mission running, initial tack=%s",
                  (tack_side_ == TackSide::STARBOARD) ? "STARBOARD" : "PORT");
    }

    // Falling edge: stop outputs
    if (!new_armed && armed_) {
      state_ = State::IDLE;
      pretack_active_ = false;
      roll_dump_active_ = false;
      RCLCPP_WARN(get_logger(), "DISARMED: mission idle.");
    }

    armed_ = new_armed;
  }

  void heading_cb(const std_msgs::msg::Float32::SharedPtr msg) {
    heading_deg_ = wrap360(msg->data);
    have_heading_ = true;
  }

  void yaw_rate_cb(const std_msgs::msg::Float32::SharedPtr msg) {
    yaw_rate_deg_s_ = msg->data;
    have_yaw_rate_ = true;
  }

  void roll_cb(const std_msgs::msg::Float32::SharedPtr msg) {
    roll_deg_ = msg->data;
    have_roll_ = true;
  }

  void awa_cb(const std_msgs::msg::Float32::SharedPtr msg) {
    awa_deg_0_360_ = wrap360(msg->data);
    have_awa_ = true;
  }

  // ---------------- Core logic ----------------

  enum class State : uint8_t { IDLE = 0, RUN = 1 };
  enum class TackSide : uint8_t { PORT = 0, STARBOARD = 1 };

  float compute_twd_deg() const {
    // AWA 0..360 => signed -180..180
    const float awa_signed = wrap180(awa_deg_0_360_);
    // Approx TWD = heading + AWA
    return wrap360(heading_deg_ + awa_signed);
  }

  float close_hauled_heading_for_tack(float twd_deg, TackSide side) const {
    const float ng = static_cast<float>(no_go_deg_);
    if (side == TackSide::STARBOARD) {
      return wrap360(twd_deg + ng);
    } else {
      return wrap360(twd_deg - ng);
    }
  }

  void update_roll_dump(const rclcpp::Time& t_now) {
    if (!have_roll_) return;

    const float roll_abs = std::fabs(roll_deg_);

    if (!roll_dump_active_) {
      if (roll_abs >= static_cast<float>(roll_dump_trip_deg_)) {
        if (!roll_trip_start_valid_) {
          roll_trip_start_valid_ = true;
          roll_trip_start_time_ = t_now;
        } else if ((t_now - roll_trip_start_time_).seconds() >= roll_trip_sec_) {
          roll_dump_active_ = true;
          roll_clear_start_valid_ = false;
          RCLCPP_WARN(get_logger(), "ROLL DUMP ACTIVE (|roll|=%.1f deg)", roll_abs);
        }
      } else {
        roll_trip_start_valid_ = false;
      }
    } else {
      // Active: clear with hysteresis and clear-time
      if (roll_abs <= static_cast<float>(roll_dump_clear_deg_)) {
        if (!roll_clear_start_valid_) {
          roll_clear_start_valid_ = true;
          roll_clear_start_time_ = t_now;
        } else if ((t_now - roll_clear_start_time_).seconds() >= roll_clear_sec_) {
          roll_dump_active_ = false;
          roll_trip_start_valid_ = false;
          RCLCPP_INFO(get_logger(), "ROLL DUMP CLEARED (|roll|=%.1f deg)", roll_abs);
        }
      } else {
        roll_clear_start_valid_ = false;
      }
    }
  }

  bool yaw_authority_ok(float expected_turn_sign, const rclcpp::Time& t_now) {
    // expected_turn_sign: +1 means we expect heading to increase, -1 means decrease (wrap-aware check below)
    if (!have_heading_ || !have_yaw_rate_) return false;

    // Condition 1: yaw rate magnitude
    const bool yaw_ok = (std::fabs(yaw_rate_deg_s_) >= yaw_rate_pub_);

    // Condition 2: heading moved in intended direction by at least heading_delta_min_deg_
    // Use signed shortest error from start heading to current heading.
    const float delta = shortest_error_deg(heading_deg_, pretack_start_heading_);
    // If expected sign positive, delta should be positive, etc.
    const bool dir_ok = (signf(delta) == expected_turn_sign) &&
                        (std::fabs(delta) >= heading_delta_min_deg_);

    // Also allow success if the yaw rate sign matches intention AND delta is already large enough.
    (void)t_now;
    return yaw_ok && dir_ok;
  }

  void maybe_start_pretack(const rclcpp::Time& t_now) {
    // Time-based tack attempt
    const double since_commit = (t_now - last_tack_commit_time_).seconds();
    if (since_commit < tack_period_sec_) return;

    // Throttle retries
    if (last_retry_time_.nanoseconds() != 0) {
      const double since_retry = (t_now - last_retry_time_).seconds();
      if (since_retry < retry_interval_sec_) return;
    }

    // Begin pretack gate
    pretack_active_ = true;
    pretack_start_time_ = t_now;
    pretack_start_heading_ = heading_deg_;
    last_retry_time_ = t_now;

    // Intend to switch to opposite tack (but do not commit yet)
    intended_tack_side_ = (tack_side_ == TackSide::STARBOARD) ? TackSide::PORT : TackSide::STARBOARD;

    RCLCPP_INFO(get_logger(), "PRETACK start: intending %s tack",
                (intended_tack_side_ == TackSide::STARBOARD) ? "STARBOARD" : "PORT");
  }

  void step() {
    const auto t_now = now();

    // Always publish sheet override (0 or 100) when armed; if disarmed, publish 0.
    update_roll_dump(t_now);
    publish_sheet_override();

    if (!armed_ || state_ == State::IDLE) {
      // Do not publish desired heading while disarmed (control will hold current heading by its own arming behavior).
      return;
    }

    // Require minimum sensor set to compute headings
    if (!have_heading_ || !have_awa_) {
      return;
    }

    const float twd = compute_twd_deg();

    // Determine nominal close-hauled heading for current tack side
    float desired = close_hauled_heading_for_tack(twd, tack_side_);

    // Timed tack logic with yaw authority gate
    if (!pretack_active_) {
      maybe_start_pretack(t_now);
    }

    if (pretack_active_) {
      // During pretack, command a small offset in the intended turn direction to check authority.
      // Choose turn direction based on which tack we're switching to:
      // STARBOARD tack generally implies desired heading is on + side of TWD, etc.
      // But for authority gating, simplest is: turn toward the *new* target heading and use sign of error.
      const float new_target = close_hauled_heading_for_tack(twd, intended_tack_side_);
      const float err_to_new = shortest_error_deg(new_target, heading_deg_);
      const float expected_turn_sign = (err_to_new >= 0.0f) ? +1.0f : -1.0f;

      // Command a small step in that direction
      desired = wrap360(heading_deg_ + static_cast<float>(pretack_offset_deg_ * expected_turn_sign));

      // Check window expiry / success
      const double pretack_elapsed = (t_now - pretack_start_time_).seconds();

      if (yaw_authority_ok(expected_turn_sign, t_now)) {
        // Commit tack
        tack_side_ = intended_tack_side_;
        pretack_active_ = false;
        last_tack_commit_time_ = t_now;
        RCLCPP_INFO(get_logger(), "TACK COMMITTED: now %s tack",
                    (tack_side_ == TackSide::STARBOARD) ? "STARBOARD" : "PORT");
      } else if (pretack_elapsed > response_window_sec_) {
        // Failed authority check -> abandon pretack, do NOT switch tack. Will retry later.
        pretack_active_ = false;
        RCLCPP_WARN(get_logger(),
                    "PRETACK failed authority gate (elapsed=%.2fs). Will retry.",
                    pretack_elapsed);
      }
    }

    // Publish desired heading at fixed rate
    std_msgs::msg::Float32 out;
    out.data = wrap360(desired);
    desired_heading_pub_->publish(out);
  }

  void publish_sheet_override() {
    std_msgs::msg::Float32 msg;
    if (!armed_) {
      msg.data = 0.0f;
    } else {
      msg.data = roll_dump_active_ ? 100.0f : 0.0f;
    }
    sheet_override_pub_->publish(msg);
  }

  // ---------------- Members ----------------

  // Parameters
  double rate_hz_{50.0};
  double no_go_deg_{40.0};
  double tack_period_sec_{60.0};
  double pretack_offset_deg_{10.0};

  double yaw_rate_pub_{5.0};
  double heading_delta_min_deg_{10.0};
  double response_window_sec_{2.0};
  double retry_interval_sec_{3.0};

  double roll_dump_trip_deg_{40.0};
  double roll_dump_clear_deg_{35.0};
  double roll_trip_sec_{0.3};
  double roll_clear_sec_{1.0};

  // Topics
  std::string armed_topic_;
  std::string heading_topic_;
  std::string yaw_rate_topic_;
  std::string roll_topic_;
  std::string awa_topic_;
  std::string desired_heading_pub_topic_;
  std::string sheet_override_pub_topic_;

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr armed_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr heading_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_rate_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr roll_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr awa_sub_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr desired_heading_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sheet_override_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // State
  bool armed_{false};
  State state_{State::IDLE};

  float heading_deg_{0.0f};
  bool have_heading_{false};

  float yaw_rate_deg_s_{0.0f};
  bool have_yaw_rate_{false};

  float roll_deg_{0.0f};
  bool have_roll_{false};

  float awa_deg_0_360_{0.0f};
  bool have_awa_{false};

  TackSide tack_side_{TackSide::STARBOARD};

  // Tack gating
  bool pretack_active_{false};
  TackSide intended_tack_side_{TackSide::PORT};
  rclcpp::Time last_tack_commit_time_{0,0,RCL_ROS_TIME};
  rclcpp::Time pretack_start_time_{0,0,RCL_ROS_TIME};
  float pretack_start_heading_{0.0f};
  rclcpp::Time last_retry_time_{0,0,RCL_ROS_TIME};

  // Roll dump debounce/hysteresis
  bool roll_dump_active_{false};
  bool roll_trip_start_valid_{false};
  rclcpp::Time roll_trip_start_time_{0,0,RCL_ROS_TIME};

  bool roll_clear_start_valid_{false};
  rclcpp::Time roll_clear_start_time_{0,0,RCL_ROS_TIME};
};

}  // namespace sailbot

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sailbot::MissionNode>());
  rclcpp::shutdown();
  return 0;
}
