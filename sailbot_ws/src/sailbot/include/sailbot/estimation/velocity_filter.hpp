#pragma once

namespace sailbot::estimation {

/// Simple scalar Kalman filter for smoothing a 1D signal (e.g., speed).
/// This is a standard constant-value model:
///   x_k = x_{k-1}
///   z_k = x_k + v_k
///
/// You give it process noise Q and measurement noise R.
class ScalarKalmanFilter {
public:
  ScalarKalmanFilter() = default;

  ScalarKalmanFilter(float initial_value,
                     float initial_variance,
                     float process_noise_q,
                     float measurement_noise_r);

  /// Initialize / reset the filter.
  void reset(float initial_value,
             float initial_variance,
             float process_noise_q,
             float measurement_noise_r);

  /// Update with a new measurement z.
  /// Returns the posterior (filtered) estimate.
  float update(float z);

  bool is_initialized() const { return initialized_; }
  float value() const { return x_; }
  float variance() const { return P_; }

private:
  bool initialized_ = false;
  float x_ = 0.0f;  // state estimate
  float P_ = 1.0f;  // estimate variance
  float Q_ = 1e-3f; // process noise
  float R_ = 1e-2f; // measurement noise
};

}  // namespace sailbot::estimation
