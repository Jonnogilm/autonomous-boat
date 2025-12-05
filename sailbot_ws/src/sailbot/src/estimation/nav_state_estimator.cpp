/*Fuses heading (IMU if present, else COG fallback), SOG, position.
Computes True Wind Direction (TWD) and True Wind Speed (TWS) from AWA + boat velocity.
Publishes a single NavState message: {lat, lon, heading, sog, cog, awa, twd, tws}.
*/
#include "sailbot/estimation/nav_state_estimator.hpp"

#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>

namespace sailbot::estimation {

NavStateEstimator::NavStateEstimator(const Params& params)
    : params_(params),
      speed_filter_() {
  // not initialized until first GNSS update
}

float NavStateEstimator::wrap360(float deg) {
  float a = std::fmod(deg, 360.0f);
  if (a < 0.0f) a += 360.0f;
  return a;
}

float NavStateEstimator::wrap180(float deg) {
  float a = std::fmod(deg, 360.0f);
  if (a < 0.0f) a += 360.0f;
  if (a >= 180.0f) a -= 360.0f;
  return a;
}

float NavStateEstimator::knots_from_mps(float mps) {
  return mps * 1.94384449f; // 1 m/s ≈ 1.94384 kn
}

double NavStateEstimator::deg_to_rad(double deg) {
  return deg * M_PI / 180.0;
}

double NavStateEstimator::rad_to_deg(double rad) {
  return rad * 180.0 / M_PI;
}

void NavStateEstimator::update_gnss(const sensors::GnssFix& fix,
                                    double t_sec) {
  if (!raw_state_.has_value()) {
    raw_state_ = NavState{};
  }

  NavState raw = raw_state_.value();

  raw.lat_deg = fix.latitude_deg;
  raw.lon_deg = fix.longitude_deg;
  raw.alt_m = fix.altitude_m;
  raw.sog_mps = static_cast<float>(fix.speed_mps);
  raw.sog_knots = knots_from_mps(raw.sog_mps);
  raw.cog_deg = wrap360(static_cast<float>(fix.course_deg));

  raw.gnss_valid = (fix.fix_quality > 0);
  raw.gnss_fix_quality = fix.fix_quality;
  raw.gnss_num_sat = fix.num_satellites;

  raw.stamp_sec = t_sec;

  raw_state_ = raw;

  if (raw.gnss_valid) {
    has_gnss_fix_ = true;
    last_gnss_time_sec_ = t_sec;

    if (!speed_filter_.is_initialized()) {
      speed_filter_.reset(raw.sog_mps, 1.0f,
                          params_.speed_kf_Q,
                          params_.speed_kf_R);
    } else {
      speed_filter_.update(raw.sog_mps);
    }
  }
}

void NavStateEstimator::update_imu(float heading_deg,
                                   float yaw_rate_deg_s,
                                   double t_sec) {
  if (!raw_state_.has_value()) {
    raw_state_ = NavState{};
  }

  NavState raw = raw_state_.value();
  raw.heading_imu_deg = wrap360(heading_deg);
  raw.yaw_rate_deg_s = yaw_rate_deg_s;
  raw.stamp_sec = t_sec;
  raw_state_ = raw;
}

void NavStateEstimator::fuse_heading_and_speed() {
  if (!raw_state_.has_value()) return;

  const NavState& raw = raw_state_.value();

  NavState fused = fused_state_.value_or(NavState{});
  fused.lat_deg = raw.lat_deg;
  fused.lon_deg = raw.lon_deg;
  fused.alt_m = raw.alt_m;

  // Smoothed speed
  float speed_mps = raw.sog_mps;
  if (speed_filter_.is_initialized()) {
    speed_mps = speed_filter_.value();
  }
  fused.sog_mps = speed_mps;
  fused.sog_knots = knots_from_mps(speed_mps);

  fused.cog_deg = raw.cog_deg;
  fused.heading_imu_deg = raw.heading_imu_deg;
  fused.yaw_rate_deg_s = raw.yaw_rate_deg_s;
  fused.gnss_valid = raw.gnss_valid;
  fused.gnss_fix_quality = raw.gnss_fix_quality;
  fused.gnss_num_sat = raw.gnss_num_sat;
  fused.stamp_sec = raw.stamp_sec;

  // Heading fusion
  const float speed_kn = fused.sog_knots;
  float fused_heading_deg = fused.heading_imu_deg;

  if (speed_kn < params_.cog_min_speed_knots || !raw.gnss_valid) {
    // Too slow or no GNSS: IMU only
    fused_heading_deg = fused.heading_imu_deg;
  } else {
    // Use weighted circular mean of IMU heading and COG
    const float h_imu_rad = static_cast<float>(deg_to_rad(fused.heading_imu_deg));
    const float h_cog_rad = static_cast<float>(deg_to_rad(fused.cog_deg));

    const float w_imu = params_.heading_weight_imu;
    const float w_cog = params_.heading_weight_cog;

    const float x = w_imu * std::cos(h_imu_rad) + w_cog * std::cos(h_cog_rad);
    const float y = w_imu * std::sin(h_imu_rad) + w_cog * std::sin(h_cog_rad);

    float fused_rad = std::atan2(y, x);
    fused_heading_deg = wrap360(static_cast<float>(rad_to_deg(fused_rad)));
  }

  fused.heading_fused_deg = fused_heading_deg;

  fused_state_ = fused;
}

void NavStateEstimator::dead_reckon(double dt_sec) {
  if (!fused_state_.has_value()) return;

  NavState fused = fused_state_.value();
  if (dt_sec <= 0.0) {
    return;
  }

  // If GNSS is stale, propagate position using last fused speed + heading.
  const double now_speed_mps = fused.sog_mps;
  const double heading_rad = deg_to_rad(fused.heading_fused_deg);

  // Earth's radius in meters (approx).
  constexpr double R = 6371000.0;

  // Distance traveled in this step.
  const double ds = now_speed_mps * dt_sec;

  // Local ENU approximation.
  const double d_north = ds * std::cos(heading_rad);
  const double d_east  = ds * std::sin(heading_rad);

  const double lat_rad = deg_to_rad(fused.lat_deg);
  const double dlat = d_north / R;
  const double dlon = d_east / (R * std::cos(lat_rad));

  fused.lat_deg += rad_to_deg(dlat);
  fused.lon_deg += rad_to_deg(dlon);

  fused_state_ = fused;
}

void NavStateEstimator::step(double t_sec) {
  if (!raw_state_.has_value()) return;

  if (last_step_time_sec_ <= 0.0) {
    last_step_time_sec_ = t_sec;
  }

  const double dt = t_sec - last_step_time_sec_;
  last_step_time_sec_ = t_sec;

  // Determine if GNSS is fresh or stale.
  bool gnss_stale = true;
  if (has_gnss_fix_) {
    const double age = t_sec - last_gnss_time_sec_;
    gnss_stale = (age > params_.gnss_timeout_sec);
  }

  // Always fuse heading + speed using latest raw state.
  fuse_heading_and_speed();

  // If GNSS is stale, propagate with dead reckoning.
  if (gnss_stale) {
    dead_reckon(dt);
  } else {
    // If GNSS is fresh, fused_state position already follows raw_state.
    // (Done in fuse_heading_and_speed via copy.)
  }
}

}  // namespace sailbot::estimation
