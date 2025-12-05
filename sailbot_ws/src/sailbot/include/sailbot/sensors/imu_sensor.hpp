#pragma once

#include <optional>

namespace sailbot::sensors {

/// IMU view coming from the Teseo-VIC3D (or separate IMU later).
/// Right now this is a placeholder that will later be updated by parsing
/// INS / DR messages from the GNSS/IMU.
class ImuSensor {
public:
  ImuSensor() = default;

  // Update functions will eventually be called from a parser that reads
  // Teseo's NMEA/DR messages.
  void set_yaw_deg(float yaw_deg) { yaw_deg_ = yaw_deg; }
  void set_yaw_rate_deg_s(float yaw_rate_deg_s) { yaw_rate_deg_s_ = yaw_rate_deg_s; }

  bool has_yaw() const { return yaw_deg_.has_value(); }
  bool has_yaw_rate() const { return yaw_rate_deg_s_.has_value(); }

  float yaw_deg() const { return yaw_deg_.value_or(0.0f); }
  float yaw_rate_deg_s() const { return yaw_rate_deg_s_.value_or(0.0f); }

private:
  std::optional<float> yaw_deg_;
  std::optional<float> yaw_rate_deg_s_;
};

}  // namespace sailbot::sensors
