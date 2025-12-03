#include "sailbot/control/sheet_controller.hpp"
#include <algorithm>
#include <cmath>

namespace sailbot::control {

SheetController::SheetController(const Params& params)
: params_(params) {}

float SheetController::compute_from_awa(float awa_deg) const {
  // Work with absolute angle off the bow: 0..180
  float awa_abs = std::fabs(awa_deg);
  if (awa_abs > 180.0f) {
    // normalize to [-180,180]
    awa_abs = std::fmod(awa_abs, 360.0f);
    if (awa_abs > 180.0f) awa_abs = 360.0f - awa_abs;
  }

  // Head-to-wind and deep in no-go: sheet basically in
  if (awa_abs <= params_.close_hauled_awa_deg) {
    return params_.close_hauled_pct;
  }

  // Piecewise linear interpolation between key trim points:
  // close-hauled -> beam reach -> broad reach -> run

  const float ch = params_.close_hauled_awa_deg;
  const float br = params_.beam_reach_awa_deg;
  const float brd = params_.broad_reach_awa_deg;
  const float run = params_.run_awa_deg;

  float pct = params_.run_pct;

  if (awa_abs <= br) {
    // between close-hauled and beam reach
    float t = (awa_abs - ch) / (br - ch);
    pct = params_.close_hauled_pct +
          t * (params_.beam_reach_pct - params_.close_hauled_pct);
  } else if (awa_abs <= brd) {
    // between beam reach and broad reach
    float t = (awa_abs - br) / (brd - br);
    pct = params_.beam_reach_pct +
          t * (params_.broad_reach_pct - params_.beam_reach_pct);
  } else if (awa_abs <= run) {
    // between broad reach and run
    float t = (awa_abs - brd) / (run - brd);
    pct = params_.broad_reach_pct +
          t * (params_.run_pct - params_.broad_reach_pct);
  } else {
    // deeper than run_awa_deg: just hold run trim
    pct = params_.run_pct;
  }

  // Enforce global min/max
  pct = std::clamp(pct, params_.min_sheet_pct, params_.max_sheet_pct);
  return pct;
}

} // namespace sailbot::control
