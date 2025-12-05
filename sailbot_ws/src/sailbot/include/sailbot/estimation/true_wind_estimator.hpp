#pragma once

#include <optional>

namespace sailbot::estimation {

struct ApparentWindState {
  // Apparent wind angle relative to boat heading, in degrees.
  //   0   = wind from directly ahead
  //  180  = from directly behind
  //  +90  = on starboard beam
  //  -90  = on port beam
  float awa_deg = 0.0f;
  bool valid = false;
};

/// For now, only estimates apparent wind angle (AWA) using:
///  - raw vane angle in radians (aligned with bow during init)
///  - fused boat heading in degrees (0..360)
///
/// Once you have an apparent wind speed sensor, we can extend this
/// to estimate true wind speed and direction.
class TrueWindEstimator {
public:
  TrueWindEstimator() = default;

  /// Update with latest sensor values.
  /// @param vane_angle_rad raw AS5600 angle [0..2*pi)
  /// @param heading_deg    fused boat heading [0..360)
  void update(float vane_angle_rad, float heading_deg);

  ApparentWindState apparent() const { return awa_state_; }

private:
  ApparentWindState awa_state_;

  static float rad_to_deg(float rad);
  static float wrap180(float deg);
};

}  // namespace sailbot::estimation
