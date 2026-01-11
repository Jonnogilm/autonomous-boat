#include "sailbot/sensors/wind_sensor.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

extern "C" {
#include <lgpio.h>
}

namespace sailbot::sensors {

namespace {

// From your Python
constexpr uint8_t CW_TRANSITIONS[]  = {0b0001, 0b0111, 0b1110, 0b1000};
constexpr uint8_t CCW_TRANSITIONS[] = {0b0010, 0b1011, 0b1101, 0b0100};

static bool in_set(uint8_t v, const uint8_t* set, size_t n) {
  for (size_t i = 0; i < n; i++) {
    if (set[i] == v) return true;
  }
  return false;
}

static float wrap360f(float deg) {
  while (deg >= 360.0f) deg -= 360.0f;
  while (deg < 0.0f) deg += 360.0f;
  return deg;
}

}  // namespace

WindSensor::WindSensor(const Params& params)
: params_(params) {}

WindSensor::~WindSensor() {
  stop();
}

void WindSensor::start() {
  if (running_.exchange(true)) return;
  worker_ = std::thread(&WindSensor::run, this);
}

void WindSensor::stop() {
  if (!running_.exchange(false)) return;
  if (worker_.joinable()) worker_.join();
}

float WindSensor::angle_deg() const {
  const int ticks_per_rev = std::max(1, params_.detents_per_rev * 4);
  const int ticks = position_ticks_.load() - zero_offset_ticks_.load();
  float deg = 360.0f * (static_cast<float>(ticks) / static_cast<float>(ticks_per_rev));
  return wrap360f(deg);
}

void WindSensor::zero_here() {
  zero_offset_ticks_.store(position_ticks_.load());
}

bool WindSensor::consume_button_pressed() {
  // returns true once per press
  return button_pressed_.exchange(false);
}

void WindSensor::run() {
  chip_handle_ = lgGpiochipOpen(params_.gpiochip);
  if (chip_handle_ < 0) {
    running_.store(false);
    throw std::runtime_error("WindSensor: failed to open gpiochip");
  }

  if (lgGpioClaimInput(chip_handle_, 0, params_.gpio_clk) < 0 ||
      lgGpioClaimInput(chip_handle_, 0, params_.gpio_dt)  < 0 ||
      lgGpioClaimInput(chip_handle_, 0, params_.gpio_sw)  < 0) {
    lgGpiochipClose(chip_handle_);
    chip_handle_ = -1;
    running_.store(false);
    throw std::runtime_error("WindSensor: failed to claim gpio inputs");
  }

  int clk = lgGpioRead(chip_handle_, params_.gpio_clk);
  int dt  = lgGpioRead(chip_handle_, params_.gpio_dt);
  last_state_ = static_cast<uint8_t>(((clk & 1) << 1) | (dt & 1));

  int last_sw = lgGpioRead(chip_handle_, params_.gpio_sw);

  const auto debounce = std::chrono::duration<double>(params_.debounce_sec);
  auto last_step_time = std::chrono::steady_clock::now();

  while (running_.load()) {
    clk = lgGpioRead(chip_handle_, params_.gpio_clk);
    dt  = lgGpioRead(chip_handle_, params_.gpio_dt);
    uint8_t state = static_cast<uint8_t>(((clk & 1) << 1) | (dt & 1));

    if (state != last_state_) {
      auto now = std::chrono::steady_clock::now();
      if ((now - last_step_time) >= debounce) {
        uint8_t transition = static_cast<uint8_t>((last_state_ << 2) | state);
        bool cw  = in_set(transition, CW_TRANSITIONS, 4);
        bool ccw = in_set(transition, CCW_TRANSITIONS, 4);

        if (cw || ccw) {
          int delta = cw ? 1 : -1;
          if (params_.invert) delta = -delta;
          position_ticks_.fetch_add(delta);
          last_step_time = now;
        }
      }
      last_state_ = state;
    }

    // Button active-low
    int sw = lgGpioRead(chip_handle_, params_.gpio_sw);
    if (sw == 0 && last_sw == 1) {
      // Your requirement: set current angle to 0°
      zero_here();
      // Signal event for node to publish /system/armed
      button_pressed_.store(true);
    }
    last_sw = sw;

    std::this_thread::sleep_for(std::chrono::microseconds(500));
  }

  lgGpiochipClose(chip_handle_);
  chip_handle_ = -1;
}

}  // namespace sailbot::sensors
