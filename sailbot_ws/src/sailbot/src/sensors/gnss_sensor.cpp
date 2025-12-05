//Publishes lat/lon, SOG/COG, fix quality, time.
#include "sailbot/sensors/gnss_sensor.hpp"

#include <charconv>
#include <cstring>
#include <sstream>
#include <vector>

namespace sailbot::sensors {

namespace {

// Split "A,B,C" into {"A","B","C"}
std::vector<std::string> split_csv(const std::string& s) {
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ',')) {
    out.push_back(item);
  }
  return out;
}

}  // namespace

void GnssSensor::handle_nmea_sentence(const std::string& sentence) {
  if (sentence.size() < 6 || sentence[0] != '$') {
    return;
  }

  // Strip leading '$' and trailing CR/LF
  std::string s = sentence;
  if (!s.empty() && (s.back() == '\n' || s.back() == '\r')) s.pop_back();
  if (!s.empty() && (s.back() == '\r')) s.pop_back();

  // Find checksum separator
  auto star_pos = s.find('*');
  std::string without_cs = (star_pos == std::string::npos) ? s.substr(1) : s.substr(1, star_pos - 1);

  // Sentence type is first 5 chars after '$', e.g. "GPGGA", "GPRMC"
  if (without_cs.size() < 5) return;
  std::string type = without_cs.substr(0, 5);
  std::string body = without_cs.substr(6);  // skip "GPGGA," etc.

  if (type == "GPGGA" || type == "GNGGA") {
    parse_gga(body);
  } else if (type == "GPRMC" || type == "GNRMC") {
    parse_rmc(body);
  }
}

void GnssSensor::parse_gga(const std::string& body) {
  auto fields = split_csv(body);
  if (fields.size() < 10) return;

  // GGA: time, lat, N/S, lon, E/W, fix, num_sat, hdop, alt, ...
  const std::string& lat_str = fields[1];
  const std::string& lat_hemi = fields[2];
  const std::string& lon_str = fields[3];
  const std::string& lon_hemi = fields[4];
  const std::string& fix_q = fields[5];
  const std::string& num_sat = fields[6];
  const std::string& alt_str = fields[8];

  GnssFix fix = last_fix_.value_or(GnssFix{});

  if (!lat_str.empty() && !lat_hemi.empty()) {
    fix.latitude_deg = parse_lat_lon_deg(lat_str, lat_hemi[0]);
  }
  if (!lon_str.empty() && !lon_hemi.empty()) {
    fix.longitude_deg = parse_lat_lon_deg(lon_str, lon_hemi[0]);
  }
  if (!alt_str.empty()) {
    fix.altitude_m = std::stod(alt_str);
  }
  if (!fix_q.empty()) {
    fix.fix_quality = std::stoi(fix_q);
  }
  if (!num_sat.empty()) {
    fix.num_satellites = std::stoi(num_sat);
  }

  last_fix_ = fix;
}

void GnssSensor::parse_rmc(const std::string& body) {
  auto fields = split_csv(body);
  if (fields.size() < 8) return;

  // RMC: time, status, lat, N/S, lon, E/W, sog(knots), cog, ...
  const std::string& status = fields[1];
  const std::string& lat_str = fields[2];
  const std::string& lat_hemi = fields[3];
  const std::string& lon_str = fields[4];
  const std::string& lon_hemi = fields[5];
  const std::string& sog_knots_str = fields[6];
  const std::string& cog_str = fields[7];

  if (status.empty() || status[0] != 'A') {
    // A = active, V = void/no fix
    return;
  }

  GnssFix fix = last_fix_.value_or(GnssFix{});

  if (!lat_str.empty() && !lat_hemi.empty()) {
    fix.latitude_deg = parse_lat_lon_deg(lat_str, lat_hemi[0]);
  }
  if (!lon_str.empty() && !lon_hemi.empty()) {
    fix.longitude_deg = parse_lat_lon_deg(lon_str, lon_hemi[0]);
  }
  if (!sog_knots_str.empty()) {
    double sog_knots = std::stod(sog_knots_str);
    fix.speed_mps = sog_knots * 0.514444;  // knots → m/s
  }
  if (!cog_str.empty()) {
    fix.course_deg = std::stod(cog_str);
  }

  last_fix_ = fix;
}

double GnssSensor::parse_lat_lon_deg(const std::string& field, char hemi) {
  // NMEA format: lat = ddmm.mmmm, lon = dddmm.mmmm
  if (field.size() < 4) return 0.0;

  // Split into degrees and minutes
  std::size_t dot_pos = field.find('.');
  std::size_t deg_len = (dot_pos == std::string::npos) ? field.size() - 2 : dot_pos - 2;
  std::string deg_part = field.substr(0, deg_len);
  std::string min_part = field.substr(deg_len);

  double deg = std::stod(deg_part);
  double minutes = std::stod(min_part);
  double value = deg + minutes / 60.0;

  if (hemi == 'S' || hemi == 'W') value = -value;
  return value;
}

}  // namespace sailbot::sensors
