#pragma once

#include <optional>
#include <string>

namespace sailbot::sensors {

/// Parsed GNSS state (from NMEA coming off the Teseo-VIC3D).
struct GnssFix {
  double latitude_deg = 0.0;
  double longitude_deg = 0.0;
  double altitude_m = 0.0;
  double speed_mps = 0.0;
  double course_deg = 0.0;
  int fix_quality = 0;    // 0 = invalid, 1 = GPS, 2 = DGPS, etc.
  int num_satellites = 0;
};

/// Simple GNSS sensor that accumulates parsed NMEA into a GnssFix object.
/// Right now this is mostly skeleton; actual parsing can be added incrementally.
class GnssSensor {
public:
  GnssSensor() = default;

  /// Feed a single NMEA sentence (e.g. "$GPGGA,...", "$GPRMC,...").
  /// This will update internal state if it is a recognized sentence type.
  void handle_nmea_sentence(const std::string& sentence);

  /// Returns the last known fix, if any.
  std::optional<GnssFix> last_fix() const { return last_fix_; }

private:
  std::optional<GnssFix> last_fix_;

  void parse_gga(const std::string& body);
  void parse_rmc(const std::string& body);

  static double parse_lat_lon_deg(const std::string& field, char hemi);
};

}  // namespace sailbot::sensors
