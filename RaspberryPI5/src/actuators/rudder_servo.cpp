/*Translate degrees / sheet percent → PWM µs (e.g., 1000–2000).
Rate-limit to avoid shock loads; enforce end-stops.
*/
#include "sailbot/actuators/rudder_servo.hpp"
#include <algorithm>

RudderServo::RudderServo(PCA9685& pca, uint8_t channel,
                         float min_us, float max_us, float center_us,
                         float min_deg, float max_deg, float rate_deg_per_s)
: pca_(pca), ch_(channel),
  min_us_(min_us), max_us_(max_us), center_us_(center_us),
  min_deg_(min_deg), max_deg_(max_deg), rate_deg_s_(rate_deg_per_s) {}

void RudderServo::set_target_deg(float deg) {
  tgt_deg_ = std::clamp(deg, min_deg_, max_deg_);
}

float RudderServo::deg_to_us(float deg) const {
  // map -max..+max around center_us using min/max_us bounds linearly
  // Assume 0 deg -> center_us, negative -> toward min_us, positive -> toward max_us
  float span_deg = (deg >= 0.f) ? max_deg_ : -min_deg_;
  float span_us  = (deg >= 0.f) ? (max_us_ - center_us_) : (center_us_ - min_us_);
  float frac = (span_deg > 0.f) ? (std::abs(deg) / span_deg) : 0.f;
  return (deg >= 0.f) ? center_us_ + frac * span_us
                      : center_us_ - frac * span_us;
}

void RudderServo::update(double dt_s) {
  // rate limiting
  float max_step = rate_deg_s_ * static_cast<float>(dt_s);
  float err = tgt_deg_ - cur_deg_;
  if (std::abs(err) > max_step) {
    cur_deg_ += (err > 0.f ? +max_step : -max_step);
  } else {
    cur_deg_ = tgt_deg_;
  }
  float pulse = deg_to_us(cur_deg_);
  pca_.set_pulse_us(ch_, pulse);
}
