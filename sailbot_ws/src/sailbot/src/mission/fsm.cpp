/*IDLE → ARM → SAIL → (TACK/JIBE/LOITER/HEAVE_TO) → RETURN_TO_HOME → FAULT
Entry/exit guards, timeouts, and clear transitions.
Safety hooks:
Geofence (lat/lon polygon or radius) → RTB
Loss of GNSS/heading/wind → HEAVE_TO (fail-safe)
Low battery / overcurrent → LOITER / RTB
Manual override channel → IDLE
*/
#include "sailbot/mission/fsm.hpp"

namespace sailbot::mission {

MissionFsm::MissionFsm(const MissionFsmParams& params)
    : params_(params), mode_(MissionMode::INIT) {}

MissionFsmOutput MissionFsm::step(const MissionFsmInput& in) {
  MissionFsmOutput out;
  out.mode = mode_;

  // Convenience flags
  const bool gnss_ready_now =
      in.gnss_valid && (in.gnss_fix_quality >= params_.min_fix_quality);

  const bool wp_reached_now =
      in.waypoint_reached ||
      (in.distance_to_wp_m > 0.0f &&
       in.distance_to_wp_m <= params_.wp_reached_margin_m);

  switch (mode_) {
    case MissionMode::INIT:
      // Immediately go to WAIT_GNSS on first call.
      mode_ = MissionMode::WAIT_GNSS;
      break;

    case MissionMode::WAIT_GNSS:
      if (gnss_ready_now) {
        mode_ = MissionMode::SAIL_TO_WP;
      }
      break;

    case MissionMode::SAIL_TO_WP:
      if (wp_reached_now) {
        mode_ = MissionMode::STOPPED;
      }
      break;

    case MissionMode::STOPPED:
      // Mission complete; stay stopped.
      break;
  }

  // Fill output after possible mode change
  out.mode = mode_;
  out.gnss_ready = gnss_ready_now;
  out.waypoint_reached = wp_reached_now;

  // Dump sails when stopped
  out.dump_sails = (mode_ == MissionMode::STOPPED);

  return out;
}

}  // namespace sailbot::mission
