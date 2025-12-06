/*Defines no-go zone (e.g., ±40–45° off TWD).
Provides VMG calculators (toward waypoint or upwind/downwind targets).
*/
#include "sailbot/guidance/wind_model.hpp"

#include <cmath>

namespace sailbot::guidance {

WindModel::WindModel(const WindModelParams& params)
    : params_(params) {}

float WindModel::wrap180(float deg) {
  float a = std::fmod(deg, 360.0f);
  if (a < 0.0f) a += 360.0f;
  if (a >= 180.0f) a -= 360.0f;
  return a;
}

void WindModel::update(float raw_awa_deg) {
  // Normalize incoming sample to [-180,180]
  float wrapped = wrap180(raw_awa_deg);

  if (!has_sample_) {
    has_sample_ = true;
    last_raw_awa_deg_ = wrapped;
    unwrapped_awa_deg_ = wrapped;
    smoothed_unwrapped_awa_deg_ = wrapped;
    smoothed_awa_deg_ = wrapped;
    return;
  }

  last_raw_awa_deg_ = wrapped;

  // Unwrap: choose +/- 360 offset so that new sample is close to previous unwrapped value.
  float candidate = wrapped;
  float diff = candidate - unwrapped_awa_deg_;
  if (diff > 180.0f) {
    candidate -= 360.0f;
  } else if (diff < -180.0f) {
    candidate += 360.0f;
  }

  unwrapped_awa_deg_ = candidate;

  // Exponential moving average on unwrapped angle.
  const float alpha = params_.awa_alpha;
  smoothed_unwrapped_awa_deg_ =
      alpha * unwrapped_awa_deg_ + (1.0f - alpha) * smoothed_unwrapped_awa_deg_;

  // Wrap back to [-180,180] for external use.
  smoothed_awa_deg_ = wrap180(smoothed_unwrapped_awa_deg_);
}

}  // namespace sailbot::guidance
