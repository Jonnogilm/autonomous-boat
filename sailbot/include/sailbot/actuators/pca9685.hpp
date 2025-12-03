#pragma once
#include <string>
#include <cstdint>

class PCA9685 {
public:
  explicit PCA9685(const std::string& i2c_dev = "/dev/i2c-1", uint8_t addr = 0x40, float osc_hz = 25'000'000.0f);
  ~PCA9685();

  bool begin();                                 // resets MODE1/MODE2, default state
  bool set_pwm_freq(float hz);                   // sets global PWM frequency
  bool set_pwm(uint8_t ch, uint16_t on, uint16_t off); // raw 12-bit
  bool set_pulse_us(uint8_t ch, float pulse_us);       // convenience: set by microseconds at current freq
  float current_freq_hz() const { return freq_hz_; }

private:
  int fd_ = -1;
  uint8_t addr_;
  float osc_hz_;
  float freq_hz_ = 50.0f;
  std::string dev_;

  bool write8(uint8_t reg, uint8_t val);
  bool read8(uint8_t reg, uint8_t& val);
  bool write_multi(uint8_t reg, const uint8_t* data, size_t len);
  uint16_t us_to_ticks(float pulse_us) const; // uses freq_hz_ & 4096
};
