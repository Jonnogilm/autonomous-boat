// to use the inland 16 channel 12-bit PWM/Servo driver via I2C.

#include "sailbot/actuators/pca9685.hpp"
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>
#include <cstring>

namespace {
constexpr uint8_t MODE1 = 0x00;
constexpr uint8_t MODE2 = 0x01;
constexpr uint8_t PRESCALE = 0xFE;
constexpr uint8_t LED0_ON_L = 0x06;
// MODE1 bits
constexpr uint8_t RESTART = 1<<7;
constexpr uint8_t SLEEP   = 1<<4;
constexpr uint8_t ALLCALL = 1<<0;
// MODE2 bits
constexpr uint8_t OUTDRV  = 1<<2;
}

PCA9685::PCA9685(const std::string& i2c_dev, uint8_t addr, float osc_hz)
: addr_(addr), osc_hz_(osc_hz), dev_(i2c_dev) {}

PCA9685::~PCA9685() { if (fd_>=0) ::close(fd_); }

bool PCA9685::begin() {
  fd_ = ::open(dev_.c_str(), O_RDWR);
  if (fd_ < 0) return false;
  if (ioctl(fd_, I2C_SLAVE, addr_) < 0) return false;

  // Reset MODE1/MODE2
  if (!write8(MODE1, ALLCALL)) return false;
  if (!write8(MODE2, OUTDRV)) return false;
  // wait for oscillator
  usleep(5000);
  // Clear sleep
  uint8_t m1;
  if (!read8(MODE1, m1)) return false;
  m1 &= ~SLEEP;
  if (!write8(MODE1, m1)) return false;
  usleep(5000);
  return set_pwm_freq(50.0f);
}

bool PCA9685::set_pwm_freq(float hz) {
  // Prescale = round(osc / (4096 * hz)) - 1
  float prescale_val = (osc_hz_ / (4096.0f * hz)) - 1.0f;
  uint8_t prescale = static_cast<uint8_t>(std::floor(prescale_val + 0.5f));
  uint8_t oldmode;
  if (!read8(MODE1, oldmode)) return false;
  uint8_t sleepmode = (oldmode & ~RESTART) | SLEEP;
  if (!write8(MODE1, sleepmode)) return false;
  if (!write8(PRESCALE, prescale)) return false;
  if (!write8(MODE1, oldmode)) return false;
  usleep(5000);
  if (!write8(MODE1, oldmode | RESTART)) return false;
  freq_hz_ = hz;
  return true;
}

bool PCA9685::set_pwm(uint8_t ch, uint16_t on, uint16_t off) {
  uint8_t buf[5];
  buf[0] = LED0_ON_L + 4 * ch;
  buf[1] = on & 0xFF;
  buf[2] = (on >> 8) & 0x0F;
  buf[3] = off & 0xFF;
  buf[4] = (off >> 8) & 0x0F;
  return write_multi(buf[0], &buf[1], 4);
}

uint16_t PCA9685::us_to_ticks(float pulse_us) const {
  // ticks = pulse_us / period_us * 4096
  const float period_us = 1'000'000.0f / freq_hz_;
  float t = (pulse_us / period_us) * 4096.0f;
  if (t < 0.0f) t = 0.0f;
  if (t > 4095.0f) t = 4095.0f;
  return static_cast<uint16_t>(std::lround(t));
}

bool PCA9685::set_pulse_us(uint8_t ch, float pulse_us) {
  auto ticks = us_to_ticks(pulse_us);
  // Use ON=0, OFF=ticks
  return set_pwm(ch, 0, ticks);
}

bool PCA9685::write8(uint8_t reg, uint8_t val) {
  uint8_t buf[2] = {reg, val};
  return (::write(fd_, buf, 2) == 2);
}
bool PCA9685::read8(uint8_t reg, uint8_t& val) {
  if (::write(fd_, &reg, 1) != 1) return false;
  return (::read(fd_, &val, 1) == 1);
}
bool PCA9685::write_multi(uint8_t reg, const uint8_t* data, size_t len) {
  uint8_t buf[1+4];
  buf[0] = reg;
  std::memcpy(&buf[1], data, len);
  return (::write(fd_, buf, 1+len) == (ssize_t)(1+len));
}
