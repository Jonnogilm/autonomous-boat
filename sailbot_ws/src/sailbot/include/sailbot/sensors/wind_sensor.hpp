#pragma once

#include <cstdint>
#include <string>

namespace sailbot::sensors {

/// Driver for an AS5600-based wind vane.
/// Reads a raw absolute angle (0..2*pi radians) from I2C.
class WindSensor {
public:
  // i2c_device: e.g. "/dev/i2c-1"
  // address:    7-bit I2C address (default 0x36)
  WindSensor(const std::string& i2c_device, uint8_t address = 0x36);

  /// Read angle in radians [0, 2*pi).
  /// Throws std::runtime_error on failure.
  float read_angle_rad(); 


private:
  int fd_;                // I2C file descriptor
  uint8_t address_;
  std::string dev_path_;

  void open_device();
  uint16_t read_raw_angle(); // 12-bit raw angle
};

}  // namespace sailbot::sensors
