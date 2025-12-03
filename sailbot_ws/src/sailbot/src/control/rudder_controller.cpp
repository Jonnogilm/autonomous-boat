#include "sailbot/control/rudder_controller.hpp"
#include <cmath>
#include <algorithm>

namespace sailbot::control {

RudderController::RudderController(const Params& params)
: params_(params) {}

float RudderController::wrap180(float angle_deg) {
  // bring into [0,360)
  float a = std::fmod(angle_deg, 360.0f);
  if (a < 0.0f) a += 360.0f;
  // then to [-180,180)
  if (a >= 180.0f) a -= 360.0f;
  return a;
}

float RudderController::compute(float desired_course_deg,
                                float heading_deg,
                                float yaw_rate_deg_s) const {
  // Compute smallest signed heading error in [-180,180)
  float error_deg = wrap180(desired_course_deg - heading_deg);

  // P + D controller: rudder = Kp * error - Kd * yaw_rate
  float rudder = params_.kp * error_deg - params_.kd * yaw_rate_deg_s;

  // Saturate to rudder limits
  rudder = std::clamp(rudder, params_.min_rudder_deg, params_.max_rudder_deg);
  return rudder;
}

} // namespace sailbot::control