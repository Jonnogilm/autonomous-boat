/*If desired course falls in no-go, selects a tack side using laylines / CTE sign and triggers TACK state when criteria met (time-on-tack, progress, or layline intercept).
Also handles jibe downwind and mark rounding behavior.
*/
#include "sailbot/guidance/tack_planner.hpp"

#include <cmath>

namespace sailbot::guidance {

TackPlanner::TackPlanner(const TackPlannerParams& params)
    : params_(params) {}

float TackPlanner::wrap360(float deg) {
  float a = std::fmod(deg, 360.0f);
  if (a < 0.0f) a += 360.0f;
  return a;
}

float TackPlanner::wrap180(float deg) {
  float a = std::fmod(deg, 360.0f);
  if (a < 0.0f) a += 360.0f;
  if (a >= 180.0f) a -= 360.0f;
  return a;
}

float TackPlanner::compute_desired_heading(float bearing_to_wp_deg,
                                           float heading_deg,
                                           float awa_deg) const {
  // Normalize inputs
  float bearing = wrap360(bearing_to_wp_deg);
  float heading = wrap360(heading_deg);
  float awa = wrap180(awa_deg);

  // Approximate true wind direction:
  //  TWD ≈ heading + AWA.
  // For AWA>0 (starboard), wind is to starboard of the bow, etc.
  float twd = wrap360(heading + awa);

  // Smallest signed angle from TWD to bearing ([-180,180))
  float diff = wrap180(bearing - twd);

  const float no_go = std::fabs(params_.no_go_angle_deg);

  // If waypoint bearing is outside the no-go cone, go straight there.
  if (std::fabs(diff) > no_go) {
    return bearing;
  }

  // Inside no-go cone: must sail close-hauled.
  // Choose tack based on which side the wind is on (AWA sign).
  float desired = heading;

  if (awa >= 0.0f) {
    // Wind on starboard => sail starboard tack (wind on right side),
    // heading slightly off the wind to port of TWD.
    desired = wrap360(twd + no_go);
  } else {
    // Wind on port => sail port tack (wind on left side),
    // heading slightly off the wind to starboard of TWD.
    desired = wrap360(twd - no_go);
  }

  return desired;
}

}  // namespace sailbot::guidance
