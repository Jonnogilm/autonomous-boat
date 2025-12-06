#pragma once

namespace sailbot::guidance {

struct WindModelParams {
  // Exponential moving average coefficient for AWA smoothing.
  // alpha in (0,1]. Smaller = smoother, more lag.
  float awa_alpha = 0.2f;
};

/// Simple wind model that just smooths apparent wind angle (AWA).
/// AWA is in degrees, [-180..180], where:
///   0   = wind from straight ahead
///   +90 = wind from starboard beam
///   -90 = wind from port beam
class WindModel {
public:
  explicit WindModel(const WindModelParams& params = WindModelParams());

  /// Feed in a new raw AWA sample (deg).
  void update(float raw_awa_deg);

  /// Returns true once at least one sample has been processed.
  bool has_awa() const { return has_sample_; }

  /// Get the last raw AWA sample (deg).
  float raw_awa_deg() const { return last_raw_awa_deg_; }

  /// Get the smoothed AWA (deg, [-180,180]).
  float smoothed_awa_deg() const { return smoothed_awa_deg_; }

private:
  WindModelParams params_;

  bool has_sample_ = false;
  float last_raw_awa_deg_ = 0.0f;

  // Internal unwrapped representation (deg).
  float unwrapped_awa_deg_ = 0.0f;
  float smoothed_unwrapped_awa_deg_ = 0.0f;

  float smoothed_awa_deg_ = 0.0f; // wrapped back to [-180,180]

  static float wrap180(float deg);
};

}  // namespace sailbot::guidance
