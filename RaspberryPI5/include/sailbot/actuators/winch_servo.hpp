#pragma once
#include "sailbot/actuators/pca9685.hpp"

class WinchServo {
public:
  WinchServo(PCA9685& pca, uint8_t channel,
             float min_us, float max_us, float neutral_us,
             float deadband_us, float rate_pct_per_s = 300.0f);

  // command as percentage speed/deflection: -100..+100 (you can feed from sheet % map)
  void set_target_pct(float pct); 
  void update(double dt_s);
  float current_pct() const { return cur_pct_; }

private:
  PCA9685& pca_;
  uint8_t ch_;
  float min_us_, max_us_, neutral_us_, deadband_us_;
  float rate_pct_s_;
  float cur_pct_ = 0.0f;
  float tgt_pct_ = 0.0f;

  float pct_to_us(float pct) const; // linear map around neutral with deadband
};
