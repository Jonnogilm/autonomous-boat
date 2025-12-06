/*Given current pose and next waypoint, computes desired course via Line-of-Sight (LOS) guidance with a look-ahead (e.g., 10–25 m) and cross-track error.
*/
#include "sailbot/guidance/waypoint_nav.hpp"

#include <cmath>

namespace sailbot::guidance {

namespace {
constexpr double EARTH_RADIUS_M = 6371000.0;
}

WaypointNavigator::WaypointNavigator(const WaypointNavParams& params)
    : params_(params) {
  ref_lat_rad_ = deg_to_rad(params_.ref_lat_deg);
  ref_lon_rad_ = deg_to_rad(params_.ref_lon_deg);
}

void WaypointNavigator::set_target(double lat_deg, double lon_deg) {
  target_lat_deg_ = lat_deg;
  target_lon_deg_ = lon_deg;
}

double WaypointNavigator::deg_to_rad(double deg) {
  return deg * M_PI / 180.0;
}

double WaypointNavigator::rad_to_deg(double rad) {
  return rad * 180.0 / M_PI;
}

float WaypointNavigator::wrap360(float deg) {
  float a = std::fmod(deg, 360.0f);
  if (a < 0.0f) a += 360.0f;
  return a;
}

void WaypointNavigator::latlon_to_enu(double lat_deg, double lon_deg,
                                      double& x_east_m,
                                      double& y_north_m) const {
  const double lat_rad = deg_to_rad(lat_deg);
  const double lon_rad = deg_to_rad(lon_deg);

  const double dlat = lat_rad - ref_lat_rad_;
  const double dlon = lon_rad - ref_lon_rad_;

  // Equirectangular projection for short distances
  y_north_m = dlat * EARTH_RADIUS_M;
  x_east_m  = dlon * std::cos(ref_lat_rad_) * EARTH_RADIUS_M;
}

WaypointNavResult WaypointNavigator::compute(double curr_lat_deg,
                                             double curr_lon_deg) const {
  double x_curr = 0.0, y_curr = 0.0;
  double x_tgt  = 0.0, y_tgt  = 0.0;

  latlon_to_enu(curr_lat_deg, curr_lon_deg, x_curr, y_curr);
  latlon_to_enu(target_lat_deg_, target_lon_deg_, x_tgt, y_tgt);

  const double dx = x_tgt - x_curr;  // east
  const double dy = y_tgt - y_curr;  // north

  WaypointNavResult res;

  const double dist = std::hypot(dx, dy);
  res.distance_m = static_cast<float>(dist);

  // Bearing: angle from north, clockwise (ENU)
  const double bearing_rad = std::atan2(dx, dy); // atan2(east, north)
  const float bearing_deg = static_cast<float>(rad_to_deg(bearing_rad));
  res.bearing_deg = wrap360(bearing_deg);

  res.target_reached = (res.distance_m <= params_.acceptance_radius_m);
  return res;
}

}  // namespace sailbot::guidance
