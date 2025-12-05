#include "sailbot/estimation/true_wind_estimator.hpp"

#define _USE_MATH_DEFINES
#include <cmath>

namespace sailbot::estimation {

float TrueWindEstimator::rad_to_deg(float rad) {
  return rad * (180.0f / static_cast<float>(M_PI));
}

float TrueWindEstimator::wrap180(float deg) {
  float a = std::fmod(deg, 360.0f);
  if (a < 0.0f) a += 360.0f;
  if (a >= 180.0f) a -= 360.0f;
  return a;
}

void TrueWindEstimator::update(float vane_angle_rad, float heading_deg) {
  // Convert vane angle to degrees (sensor reference relative to bow).
  float vane_deg = rad_to_deg(vane_angle_rad);

  // Normalize heading to [0,360)
  float h = std::fmod(heading_deg, 360.0f);
  if (h < 0.0f) h += 360.0f;

  // Apparent wind angle = vane_angle - heading (in boat frame).
  // Then wrap to [-180,180).
  float awa = wrap180(vane_deg - h);

  awa_state_.awa_deg = awa;
  awa_state_.valid = true;
}

}  // namespace sailbot::estimation
