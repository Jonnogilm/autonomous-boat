#pragma once

#include <cstdint>

namespace sailbot::guidance {

struct WaypointNavParams {
  // Reference origin for ENU (deg). Typically launch position or fixed config.
  double ref_lat_deg = 0.0;
  double ref_lon_deg = 0.0;

  // How close is "good enough" to call the waypoint reached (meters).
  float acceptance_radius_m = 5.0f;
};

struct WaypointNavResult {
  float distance_m = 0.0f;      // distance to waypoint
  float bearing_deg = 0.0f;     // bearing from current pos to waypoint [0..360)
  bool target_reached = false;  // true if within acceptance_radius_m
};

/// Simple single-waypoint navigator:
///  - Stores a target lat/lon
///  - Converts lat/lon to local ENU
///  - Computes distance and bearing to target
class WaypointNavigator {
public:
  explicit WaypointNavigator(const WaypointNavParams& params);

  /// Set the target waypoint in lat/lon (deg).
  void set_target(double lat_deg, double lon_deg);

  /// Compute distance and bearing from current position to target.
  /// @param curr_lat_deg current latitude [deg]
  /// @param curr_lon_deg current longitude [deg]
  /// @return WaypointNavResult with distance, bearing, and reached flag
  WaypointNavResult compute(double curr_lat_deg, double curr_lon_deg) const;

  /// Get the current target lat/lon (deg).
  double target_lat_deg() const { return target_lat_deg_; }
  double target_lon_deg() const { return target_lon_deg_; }

private:
  WaypointNavParams params_;

  double ref_lat_rad_;
  double ref_lon_rad_;

  double target_lat_deg_ = 0.0;
  double target_lon_deg_ = 0.0;

  // Helpers
  static double deg_to_rad(double deg);
  static double rad_to_deg(double rad);
  static float wrap360(float deg);

  // Convert lat/lon to local ENU (east, north) relative to ref origin.
  void latlon_to_enu(double lat_deg, double lon_deg,
                     double& x_east_m, double& y_north_m) const;
};

}  // namespace sailbot::guidance
