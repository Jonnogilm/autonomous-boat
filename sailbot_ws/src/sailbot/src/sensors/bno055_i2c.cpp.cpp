#include "sailbot/sensors/bno055_i2c.hpp"

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <thread>

namespace sailbot::sensors {

namespace {
inline void sleep_ms(int ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

inline std::string sys_err(const char* msg) {
  std::string s(msg);
  s += ": ";
  s += std::strerror(errno);
  return s;
}
}  // namespace

Bno055I2C::Bno055I2C(const Config& cfg) : cfg_(cfg) {}

Bno055I2C::~Bno055I2C() {
  close();
}

void Bno055I2C::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool Bno055I2C::begin(std::string* err) {
  close();

  fd_ = ::open(cfg_.i2c_dev.c_str(), O_RDWR);
  if (fd_ < 0) {
    if (err) *err = sys_err("open(i2c_dev) failed");
    return false;
  }

  if (ioctl(fd_, I2C_SLAVE, cfg_.i2c_addr) < 0) {
    if (err) *err = sys_err("ioctl(I2C_SLAVE) failed");
    close();
    return false;
  }

  // Put into CONFIG mode, reset, verify chip, set power + fusion mode.
  std::string e;
  if (!set_op_mode(MODE_CONFIG, &e)) {
    if (err) *err = "set CONFIG mode failed: " + e;
    return false;
  }

  if (!reset(&e)) {
    if (err) *err = "reset failed: " + e;
    return false;
  }

  sleep_ms(cfg_.reset_delay_ms);

  if (!verify_chip(&e)) {
    if (err) *err = "chip verify failed: " + e;
    return false;
  }

  if (!set_power_mode(PWR_NORMAL, &e)) {
    if (err) *err = "set power normal failed: " + e;
    return false;
  }

  uint8_t op = cfg_.use_ndof ? MODE_NDOF : MODE_CONFIG;
  if (!set_op_mode(op, &e)) {
    if (err) *err = "set op mode failed: " + e;
    return false;
  }

  // Give fusion engine a moment
  sleep_ms(100);

  return true;
}

bool Bno055I2C::verify_chip(std::string* err) {
  uint8_t chip = 0;
  if (!read_u8(REG_CHIP_ID, &chip, err)) {
    return false;
  }
  if (chip != 0xA0) {
    if (err) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "expected CHIP_ID 0xA0, got 0x%02X", chip);
      *err = buf;
    }
    return false;
  }
  return true;
}

bool Bno055I2C::reset(std::string* err) {
  // SYS_TRIGGER bit 5 = reset
  if (!write_u8(REG_SYS_TRIGGER, 0x20, err)) {
    return false;
  }
  sleep_ms(cfg_.write_delay_ms);
  return true;
}

bool Bno055I2C::set_op_mode(uint8_t mode, std::string* err) {
  if (!write_u8(REG_OPR_MODE, mode, err)) {
    return false;
  }
  sleep_ms(cfg_.write_delay_ms);
  return true;
}

bool Bno055I2C::set_power_mode(uint8_t mode, std::string* err) {
  if (!write_u8(REG_PWR_MODE, mode, err)) {
    return false;
  }
  sleep_ms(cfg_.write_delay_ms);
  return true;
}

std::optional<float> Bno055I2C::read_heading_deg(std::string* err) {
  // Heading is unsigned 16-bit, LSB = 1/16 degree, range [0..5760)
  uint8_t lsb = 0, msb = 0;
  if (!read_u8(REG_EULER_H_LSB, &lsb, err)) return std::nullopt;
  if (!read_u8(static_cast<uint8_t>(REG_EULER_H_LSB + 1), &msb, err)) return std::nullopt;

  uint16_t raw = static_cast<uint16_t>((msb << 8) | lsb);
  float deg = static_cast<float>(raw) / 16.0f;

  // Normalize to [0, 360)
  while (deg >= 360.0f) deg -= 360.0f;
  while (deg < 0.0f) deg += 360.0f;

  return deg;
}

std::optional<float> Bno055I2C::read_yaw_rate_deg_s(std::string* err) {
  // Gyro data is signed 16-bit, unit scaling depends on unit selection.
  // With default units, gyro is in deg/s with 16 LSB per deg/s.
  int16_t raw = 0;
  if (!read_i16_le(REG_GYR_Z_LSB, &raw, err)) return std::nullopt;

  // Default: 16 LSB per deg/s
  float deg_s = static_cast<float>(raw) / 16.0f;
  return deg_s;
}

std::optional<uint8_t> Bno055I2C::read_calib_stat(std::string* err) {
  uint8_t v = 0;
  if (!read_u8(REG_CALIB_STAT, &v, err)) return std::nullopt;
  return v;
}

std::optional<Bno055I2C::Quaternion> Bno055I2C::read_quaternion(std::string* err) {
  // Quaternion registers: W, X, Y, Z each signed 16-bit little-endian.
  // Scaling: 1 / 16384.
  uint8_t buf[8]{};
  if (!read_block(REG_QUAT_W_LSB, buf, sizeof(buf), err)) return std::nullopt;

  auto rd = [&](int i) -> int16_t {
    return static_cast<int16_t>(static_cast<uint16_t>(buf[i] | (buf[i+1] << 8)));
  };

  int16_t w_raw = rd(0);
  int16_t x_raw = rd(2);
  int16_t y_raw = rd(4);
  int16_t z_raw = rd(6);

  constexpr float scale = 1.0f / 16384.0f;

  Quaternion q;
  q.w = static_cast<float>(w_raw) * scale;
  q.x = static_cast<float>(x_raw) * scale;
  q.y = static_cast<float>(y_raw) * scale;
  q.z = static_cast<float>(z_raw) * scale;

  return q;
}

bool Bno055I2C::write_u8(uint8_t reg, uint8_t val, std::string* err) {
  uint8_t data[2] = {reg, val};
  ssize_t n = ::write(fd_, data, 2);
  if (n != 2) {
    if (err) *err = sys_err("i2c write failed");
    return false;
  }
  return true;
}

bool Bno055I2C::read_u8(uint8_t reg, uint8_t* out, std::string* err) {
  if (!out) return false;
  // Write register address, then read one byte.
  ssize_t n = ::write(fd_, &reg, 1);
  if (n != 1) {
    if (err) *err = sys_err("i2c write(reg) failed");
    return false;
  }
  n = ::read(fd_, out, 1);
  if (n != 1) {
    if (err) *err = sys_err("i2c read failed");
    return false;
  }
  return true;
}

bool Bno055I2C::read_i16_le(uint8_t reg_lsb, int16_t* out, std::string* err) {
  if (!out) return false;
  uint8_t buf[2]{};
  if (!read_block(reg_lsb, buf, 2, err)) return false;
  *out = static_cast<int16_t>(static_cast<uint16_t>(buf[0] | (buf[1] << 8)));
  return true;
}

bool Bno055I2C::read_block(uint8_t start_reg, uint8_t* buf, size_t n, std::string* err) {
  if (!buf || n == 0) return false;
  ssize_t w = ::write(fd_, &start_reg, 1);
  if (w != 1) {
    if (err) *err = sys_err("i2c write(start_reg) failed");
    return false;
  }
  ssize_t r = ::read(fd_, buf, static_cast<int>(n));
  if (r != static_cast<ssize_t>(n)) {
    if (err) *err = sys_err("i2c read(block) failed");
    return false;
  }
  return true;
}

}  // namespace sailbot::sensors
