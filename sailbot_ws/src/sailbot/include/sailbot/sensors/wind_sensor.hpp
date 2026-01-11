#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>

namespace sailbot::sensors {

class WindSensor {
public:
  struct Params {
    int gpiochip = 4;
    int gpio_clk = 16;
    int gpio_dt  = 20;
    int gpio_sw  = 21;

    int detents_per_rev = 20;
    bool invert = false;

    double debounce_sec = 0.005;
  };

  explicit WindSensor(const Params& params);
  ~WindSensor();

  void start();
  void stop();

  // Angle in degrees [0..360)
  float angle_deg() const;

  // Set current position as 0° reference
  void zero_here();

  // True once per button press; consumes the event
  bool consume_button_pressed();

private:
  void run();

  Params params_;

  std::atomic<bool> running_{false};
  std::thread worker_;

  std::atomic<int> position_ticks_{0};
  std::atomic<int> zero_offset_ticks_{0};

  // Button press event flag
  std::atomic<bool> button_pressed_{false};

  int chip_handle_{-1};
  uint8_t last_state_{0};
};

}  // namespace sailbot::sensors
