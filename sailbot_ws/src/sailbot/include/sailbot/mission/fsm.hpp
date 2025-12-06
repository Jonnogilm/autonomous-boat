#pragma once

#include <cstdint>

namespace sailbot::mission {

enum class MissionMode : uint8_t {
  INIT          = 0,
  WAIT_GNSS     = 1,
  SAIL_TO_WP    = 2,
  STOPPED       = 3,
};

struct MissionFsmParams {
  // Minimum GNSS fix quality required to consider it "locked".
  // 1 = GPS, 2 = DGPS, etc.
  int min_fix_quality = 1;

  // Distance threshold to double-check waypoint reached (meters),
  // in addition to GuidanceDebug.waypoint_reached.
  float wp_reached_margin_m = 5.0f;
};

struct MissionFsmInput {
  bool gnss_valid = false;
  int gnss_fix_quality = 0;

  bool waypoint_reached = false;
  float distance_to_wp_m = 0.0f;

  double t_sec = 0.0;  // current time (seconds)
};

struct MissionFsmOutput {
  MissionMode mode = MissionMode::INIT;
  bool gnss_ready = false;
  bool waypoint_reached = false;

  // Whether the mission wants the sails dumped (fully eased).
  bool dump_sails = false;
};

/// Simple mission FSM:
///  INIT -> WAIT_GNSS -> SAIL_TO_WP -> STOPPED
///  - WAIT_GNSS: wait until GNSS valid & fix_quality >= min_fix_quality
///  - SAIL_TO_WP: boat is under way using guidance
///  - STOPPED: waypoint reached, mission complete (dump sails)
class MissionFsm {
public:
  explicit MissionFsm(const MissionFsmParams& params = MissionFsmParams());

  /// Step FSM with new input; returns current output.
  MissionFsmOutput step(const MissionFsmInput& in);

  MissionMode mode() const { return mode_; }

private:
  MissionFsmParams params_;
  MissionMode mode_ = MissionMode::INIT;
};

}  // namespace sailbot::mission
