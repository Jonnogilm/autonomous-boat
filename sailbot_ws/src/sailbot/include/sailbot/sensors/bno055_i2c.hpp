#pragma once

#include <cstdint>
#include <string>
#include <optional>

namespace sailbot::sensors {

class Bno055I2C {
public:
  struct Config {
    std::string i2c_dev = "/dev/i2c-1";
    uint8_t i2c_addr = 0x29;

    // Timing tuned to be safe on Pi-class Linux.
    int reset_delay_ms = 700;
    int write_delay_ms = 20;

    // NDOF = fusion mode with mag/gyro/accel.
    // This is what you used in your Python script.
    bool use_ndof = true;
  };

  explicit Bno055I2C(const Config& cfg);
  ~Bno055I2C();

  Bno055I2C(const Bno055I2C&) = delete;
  Bno055I2C& operator=(const Bno055I2C&) = delete;

  bool begin(std::string* err = nullptr);
  void close();

  // Returns true if CHIP_ID == 0xA0
  bool verify_chip(std::string* err = nullptr);

  // Heading in degrees [0,360)
  std::optional<float> read_heading_deg(std::string* err = nullptr);

  // Gyro Z in deg/s (signed). Useful for D-term in heading hold.
  std::optional<float> read_yaw_rate_deg_s(std::string* err = nullptr);

  // Calibration status register (SYS/GYR/ACC/MAG nibble bits)
  std::optional<uint8_t> read_calib_stat(std::string* err = nullptr);

  // Quaternion in unitless normalized form (w,x,y,z). Optional for /imu/data publishing.
  struct Quaternion {
    float w{1.f}, x{0.f}, y{0.f}, z{0.f};
  };
  std::optional<Quaternion> read_quaternion(std::string* err = nullptr);

private:
  bool write_u8(uint8_t reg, uint8_t val, std::string* err);
  bool read_u8(uint8_t reg, uint8_t* out, std::string* err);
  bool read_i16_le(uint8_t reg_lsb, int16_t* out, std::string* err);
  bool read_block(uint8_t start_reg, uint8_t* buf, size_t n, std::string* err);

  bool set_op_mode(uint8_t mode, std::string* err);
  bool set_power_mode(uint8_t mode, std::string* err);
  bool reset(std::string* err);

  Config cfg_;
  int fd_{-1};

  // BNO055 registers (page 0)
  static constexpr uint8_t REG_CHIP_ID      = 0x00;
  static constexpr uint8_t REG_OPR_MODE     = 0x3D;
  static constexpr uint8_t REG_PWR_MODE     = 0x3E;
  static constexpr uint8_t REG_SYS_TRIGGER  = 0x3F;
  static constexpr uint8_t REG_CALIB_STAT   = 0x35;

  static constexpr uint8_t REG_EULER_H_LSB  = 0x1A;

  static constexpr uint8_t REG_GYR_Z_LSB    = 0x18; // gyro Z LSB on page 0 (X=0x14, Y=0x16, Z=0x18)

  static constexpr uint8_t REG_QUAT_W_LSB   = 0x20; // quaternion W LSB (then W MSB, X LSB..)

  // Modes
  static constexpr uint8_t MODE_CONFIG      = 0x00;
  static constexpr uint8_t MODE_NDOF        = 0x0C;

  // Power modes
  static constexpr uint8_t PWR_NORMAL       = 0x00;
};

}  // namespace sailbot::sensors
