#pragma once

#include <optional>
#include <cstdint>

#include "sailbot/sensors/gnss_sensor.hpp"
#include "sailbot/sensors/imu_sensor.hpp"
#include "sailbot/estimation/velocity_filter.hpp"

namespace sailbot::estimation {

struct NavState {
  // Position (deg)
  double lat_deg = 0.0;
  double lon_deg = 0.0;

  // Altitude (m)
  double alt_m = 0.0;

  // Speed
  float sog_mps = 0.0f;    // speed over ground [m/s]
  float sog_knots = 0.0f;  // speed over ground [knots]

  // Headings
  float cog_deg = 0.0f;      // course over ground [0..360)
  float heading_imu_deg = 0.0f;  // IMU heading [0..360)
  float heading_fused_deg = 0.0f; // fused heading [0..360)

  // Rotational
  float yaw_rate_deg_s = 0.0f;

  // GNSS status
  bool gnss_valid = false;
  int  gnss_fix_quality = 0;
  int  gnss_num_sat = 0;

  // For debugging: timestamps (sec, arbitrary monotonic time base)
  double stamp_sec = 0.0;
};

/// Fuses GNSS + IMU into a consistent NavState:
///  - Fused heading from IMU + COG
///  - Smoothed speed
///  - Dead reckoning when GNSS drops, using last known speed+heading
///  - Holds both raw and fused values.
class NavStateEstimator {
public:
  struct Params {
    float heading_weight_imu = 0.6f;
    float heading_weight_cog = 0.4f;
    float cog_min_speed_knots = 2.0f; // below this, ignore GNSS COG
    float speed_kf_Q = 1e-3f;         // process noise for speed filter
    float speed_kf_R = 5e-2f;         // measurement noise for speed filter
    double gnss_timeout_sec = 3.0;    // after this with no fix, treat as lost
  };

  explicit NavStateEstimator(const Params& params);

  /// Update GNSS fix (from GnssSensor::last_fix()) at time t_sec.
  void update_gnss(const sensors::GnssFix& fix, double t_sec);

  /// Update IMU heading and yaw-rate at time t_sec.
  /// heading_deg: [0..360), yaw_rate_deg_s: deg/s
  void update_imu(float heading_deg, float yaw_rate_deg_s, double t_sec);

  /// Step the estimator forward by dt (seconds).
  /// This does dead reckoning when GNSS is stale.
  void step(double t_sec);

  /// Latest fused state (if any).
  std::optional<NavState> fused_state() const { return fused_state_; }

  /// Latest raw state (GNSS + IMU, before fusion).
  std::optional<NavState> raw_state() const { return raw_state_; }

private:
  Params params_;

  std::optional<NavState> raw_state_;
  std::optional<NavState> fused_state_;

  ScalarKalmanFilter speed_filter_;

  bool has_gnss_fix_ = false;
  double last_gnss_time_sec_ = 0.0;
  double last_step_time_sec_ = 0.0;

  static float wrap360(float deg);
  static float wrap180(float deg);

  static float knots_from_mps(float mps);
  static double deg_to_rad(double deg);
  static double rad_to_deg(double rad);

  void fuse_heading_and_speed();
  void dead_reckon(double dt_sec);
};

}  // namespace sailbot::estimation
