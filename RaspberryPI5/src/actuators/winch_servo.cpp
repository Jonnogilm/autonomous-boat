/*Translate degrees / sheet percent → PWM µs (e.g., 1000–2000).
Rate-limit to avoid shock loads; enforce end-stops.
*/
#include "sailbot/actuators/winch_servo.hpp"
#include <algorithm>
#include <cmath>

WinchServo::WinchServo(PCA9685& pca, uint8_t channel,
                       float min_us, float max_us, float neutral_us,
                       float deadband_us, float rate_pct_per_s)
: pca_(pca), ch_(channel),
  min_us_(min_us), max_us_(max_us), neutral_us_(neutral_us),
  deadband_us_(deadband_us), rate_pct_s_(rate_pct_per_s) {}

void WinchServo::set_target_pct(float pct) {
  tgt_pct_ = std::clamp(pct, -100.0f, 100.0f);
}

float WinchServo::pct_to_us(float pct) const {
  // Map -100..+100 -> min_us..max_us, with neutral at 0 -> neutral_us
  if (std::abs(pct) < 1e-3f) {
    // inside deadband around neutral
    return neutral_us_;
  }
  float side_span_us = (pct > 0.f) ? (max_us_ - neutral_us_) : (neutral_us_ - min_us_);
  float frac = std::abs(pct) / 100.0f;
  float us = (pct > 0.f) ? neutral_us_ + frac * side_span_us
                         : neutral_us_ - frac * side_span_us;

  // Enforce neutral deadband (±deadband_us/2)
  if (pct > 0.f && us < neutral_us_ + deadband_us_/2.0f) us = neutral_us_ + deadband_us_/2.0f;
  if (pct < 0.f && us > neutral_us_ - deadband_us_/2.0f) us = neutral_us_ - deadband_us_/2.0f;
  if (pct == 0.f) us = neutral_us_;

  return std::clamp(us, min_us_, max_us_);
}

void WinchServo::update(double dt_s) {
  // rate limit
  float max_step = rate_pct_s_ * static_cast<float>(dt_s);
  float err = tgt_pct_ - cur_pct_;
  if (std::abs(err) > max_step) {
    cur_pct_ += (err > 0.f ? +max_step : -max_step);
  } else {
    cur_pct_ = tgt_pct_;
  }
  float pulse = pct_to_us(cur_pct_);
  pca_.set_pulse_us(ch_, pulse);
}
