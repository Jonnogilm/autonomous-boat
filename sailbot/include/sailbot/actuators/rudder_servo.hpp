#pragma once
#include "sailbot/actuators/pca9685.hpp"

class RudderServo {
public:
  RudderServo(PCA9685& pca, uint8_t channel,
              float min_us, float max_us, float center_us,
              float min_deg, float max_deg, float rate_deg_per_s = 180.0f);

  void set_target_deg(float deg);    // command in degrees (boat convention, e.g., -35..+35)
  void update(double dt_s);          // rate limit & write to PCA if changed
  float current_deg() const { return cur_deg_; }

private:
  PCA9685& pca_;
  uint8_t ch_;
  float min_us_, max_us_, center_us_;
  float min_deg_, max_deg_;
  float rate_deg_s_;
  float cur_deg_ = 0.0f;
  float tgt_deg_ = 0.0f;

  float deg_to_us(float deg) const;  // linear map with center at 0°
};
