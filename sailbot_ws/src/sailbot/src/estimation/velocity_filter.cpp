#include "sailbot/estimation/velocity_filter.hpp"

namespace sailbot::estimation {

ScalarKalmanFilter::ScalarKalmanFilter(float initial_value,
                                       float initial_variance,
                                       float process_noise_q,
                                       float measurement_noise_r) {
  reset(initial_value, initial_variance, process_noise_q, measurement_noise_r);
}

void ScalarKalmanFilter::reset(float initial_value,
                               float initial_variance,
                               float process_noise_q,
                               float measurement_noise_r) {
  x_ = initial_value;
  P_ = initial_variance;
  Q_ = process_noise_q;
  R_ = measurement_noise_r;
  initialized_ = true;
}

float ScalarKalmanFilter::update(float z) {
  if (!initialized_) {
    reset(z, 1.0f, Q_, R_);
    return x_;
  }

  // Predict: x_k|k-1 = x_{k-1}; P_k|k-1 = P_{k-1} + Q
  P_ += Q_;

  // Update: K = P/(P+R), x = x + K*(z-x), P = (1-K)*P
  const float S = P_ + R_;
  const float K = (S > 0.0f) ? P_ / S : 0.0f;
  const float y = z - x_;

  x_ += K * y;
  P_ = (1.0f - K) * P_;
  return x_;
}

}  // namespace sailbot::estimation
