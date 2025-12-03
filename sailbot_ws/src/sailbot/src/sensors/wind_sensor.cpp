#include "sailbot/sensors/wind_sensor.hpp"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <unistd.h>

namespace sailbot::sensors {

namespace {
constexpr float TWO_PI = 6.28318530717958647692f;
}

// AS5600 ANGLE registers: 0x0E (MSB), 0x0F (LSB), 12-bit value.
WindVaneAS5600::WindVaneAS5600(const std::string &i2c_device, uint8_t address)
    : fd_(-1), address_(address), dev_path_(i2c_device) {
  openDevice();
}

void WindVaneAS5600::openDevice() {
  fd_ = ::open(dev_path_.c_str(), O_RDWR);
  if (fd_ < 0) {
    throw std::runtime_error("WindVaneAS5600: failed to open " + dev_path_ +
                             ": " + std::strerror(errno));
  }

  if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("WindVaneAS5600: failed to set I2C_SLAVE: " +
                             std::string(std::strerror(errno)));
  }
}

uint16_t WindVaneAS5600::readRawAngle() {
  // Register address 0x0E (ANGLE high byte)
  uint8_t reg = 0x0E;
  if (write(fd_, &reg, 1) != 1) {
    throw std::runtime_error("WindVaneAS5600: failed to write angle register");
  }

  uint8_t buf[2] = {0, 0};
  ssize_t n = read(fd_, buf, 2);
  if (n != 2) {
    throw std::runtime_error("WindVaneAS5600: failed to read angle bytes");
  }

  uint16_t raw = (static_cast<uint16_t>(buf[0]) << 8) |
                 static_cast<uint16_t>(buf[1]);
  raw &= 0x0FFF; // 12-bit
  return raw;
}

float WindVaneAS5600::readAngleRad() {
  uint16_t raw = readRawAngle();
  float angle = (static_cast<float>(raw) / 4096.0f) * TWO_PI;
  return angle;
}

} // namespace sailbot::sensors
