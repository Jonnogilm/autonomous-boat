#pragma once

#include <cstdint>

namespace sailbot::control {

/// Simple LOS-style rudder controller.
/// Tracks a desired course heading using P + optional D on yaw-rate.
///
/// All angles are in degrees, using the convention:
///   - heading_deg: 0–360° (true)
///   - desired_course_deg: 0–360° (true)
///   - rudder_deg: negative = port, positive = starboard (e.g. -35..+35)
class RudderController {
public:
  struct Params {
    float kp = 1.0f;          // proportional gain [deg rudder / deg error]
    float kd = 0.0f;          // derivative gain [deg rudder / (deg/s yaw_rate)]
    float min_rudder_deg = -35.0f;
    float max_rudder_deg = +35.0f;
  };

  explicit RudderController(const Params& params = Params());

  /// Compute rudder command (deg) given desired course, current heading and optional yaw rate.
  ///
  /// @param desired_course_deg  [0..360)
  /// @param heading_deg         [0..360)
  /// @param yaw_rate_deg_s      current yaw rate (deg/s), can be 0 if no IMU yet
  /// @return rudder angle in degrees, saturated to [min_rudder_deg, max_rudder_deg]
  float compute(float desired_course_deg,
                float heading_deg,
                float yaw_rate_deg_s = 0.0f) const;

  /// Utility: wrap an angle in degrees into [-180, +180).
  static float wrap180(float angle_deg);

private:
  Params params_;
};

} // namespace sailbot::control