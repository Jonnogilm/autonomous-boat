#pragma once

namespace sailbot::guidance {

struct TackPlannerParams {
  // Half-angle of the no-go cone around the wind direction (deg).
  // e.g., 40° means you cannot sail within ±40° of the wind.
  float no_go_angle_deg = 40.0f;
};

/// Simple tacking planner based on apparent wind angle (AWA).
///
/// Inputs:
///  - bearing_to_wp_deg: desired bearing to waypoint [0..360)
///  - heading_deg:       fused boat heading [0..360)
///  - awa_deg:           apparent wind angle [-180..180],
///                        + = wind from starboard, - = from port
///
/// Output:
///  - desired heading [0..360), either:
///      * direct-to-waypoint course if outside no-go cone
///      * close-hauled course on chosen tack if waypoint is in no-go zone
class TackPlanner {
public:
  explicit TackPlanner(const TackPlannerParams& params = TackPlannerParams());

  float compute_desired_heading(float bearing_to_wp_deg,
                                float heading_deg,
                                float awa_deg) const;

private:
  TackPlannerParams params_;

  static float wrap360(float deg);
  static float wrap180(float deg);
};

}  // namespace sailbot::guidance
