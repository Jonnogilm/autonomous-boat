#pragma once

namespace sailbot::control {

/// Simple sheet controller:
/// Maps apparent wind angle (AWA) to desired sheet position in percent.
///  - 0%  = fully sheeted in
///  - 100% = fully eased out
///
/// This is deliberately simple and stateless. A more advanced version could
/// incorporate true wind speed, gust handling, or polar-based trim tables.
class SheetController {
public:
  struct Params {
    float min_sheet_pct = 0.0f;    // fully in
    float max_sheet_pct = 100.0f;  // fully out

    // Trim values at key AWA points (deg).
    float close_hauled_awa_deg = 35.0f; // near no-go edge
    float beam_reach_awa_deg   = 90.0f;
    float broad_reach_awa_deg  = 135.0f;
    float run_awa_deg          = 170.0f;

    // Corresponding sheet positions (% out)
    float close_hauled_pct = 15.0f;  // pretty tight
    float beam_reach_pct   = 55.0f;  // moderate ease
    float broad_reach_pct  = 80.0f;  // eased
    float run_pct          = 90.0f;  // almost all the way out
  };

  explicit SheetController(const Params& params = Params());

  /// Compute desired sheet position [0..100] given AWA in degrees.
  ///
  /// @param awa_deg apparent wind angle in degrees (-180..180).
  ///                0 = head to wind, positive = starboard, negative = port.
  /// @return desired sheet percentage [min_sheet_pct..max_sheet_pct]
  float compute_from_awa(float awa_deg) const;

private:
  Params params_;
};

} // namespace sailbot::control
