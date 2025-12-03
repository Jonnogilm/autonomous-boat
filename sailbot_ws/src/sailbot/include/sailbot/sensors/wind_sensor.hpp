#pragma once

#include <cstdint>
#include <string>

namespace sailbot::sensors {

class WindVaneAS5600 {
public:
  // i2c_device: e.g. "/dev/i2c-2"
  // address:    7-bit I2C address (0x36)
  WindVaneAS5600(const std::string &i2c_device, uint8_t address = 0x36);

  // Returns angle in radians [0, 2*pi).
  // Throws std::runtime_error on failure.
  float readAngleRad();

private:
  int fd_;                // I2C file descriptor
  uint8_t address_;
  std::string dev_path_;

  void openDevice();
  uint16_t readRawAngle(); // 12-bit angle
};

} // namespace sailbot::sensors
