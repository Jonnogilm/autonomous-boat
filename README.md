# autonomous-boat

> A compact open-source architecture for an autonomous sailing vessel powered by a Raspberry Pi 5, a Teseo-VIC3D GNSS+IMU, and an AS5600 magnetic wind vane.

---

## Overview

This repository contains all firmware and ROS 2 packages needed to run a self-contained autonomous sailboat.  

**Key Components**

| Subsystem | Hardware | Interface | Node(s) |
|---|---|---|---|
| Wind Vane | AS5600 magnetic encoder | I²C | `as5600_wind_node` |
| Navigation & Pose | Teseo-VIC3D (GNSS + 6-axis IMU) | UART + (PPS opt.) | `nmea_serial_driver`, `vic3d_imu_node` |
| Orientation (RPY) | Madgwick (no magnetometer) | ROS 2 | `imu_filter_madgwick` |
| Yaw Stabilization | Gyro + GPS COG fusion | ROS 2 | `heading_estimator_node` |
| Actuation | Rudder + Sail (PCA9685) | I²C PWM | `pca9685_servo_node` |
| Control | PID (heading) + LUT (sail) | ROS 2 topics | `heading_controller_node`, `sail_controller_node` |

**No magnetometer** is used. Roll/pitch come from `imu_filter_madgwick` (acc+gyro). Yaw is stabilized by a lightweight estimator that slowly corrects gyro-drift toward **GPS COG when SOG is above a threshold**.

---

## ⚙️ Repository Structure
autonomous-boat/
├─ boat_bringup/ # launch + config
├─ boat_msgs/ # custom message definitions
├─ sensors_as5600/ # masthead wind vane reader
├─ sensors_vic3d/ # GNSS + IMU wrappers
├─ estimation_heading/ # yaw: gyro + GPS COG fusion 
├─ control_heading/ # rudder PID
├─ control_sail/ # sail trim LUT with hysteresis
├─ actuators_pwm/ # servo driver + watchdog
├─ utils_watchdog/ # system safety node
└─ tools/ # calibration utilities


Each package is a standard ROS 2 node with its own `package.xml` and `CMakeLists.txt`.

---

## Hardware Wiring

| Bus | GPIO (Pi 5) | Device | Notes |
|-----|-------------|--------|-------|
| UART | 14 (TXD), 15 (RXD) | Teseo-VIC3D | 3.3 V, 38.4k–115.2k baud (fix one) |
| PPS (optional) | 4 | Teseo-VIC3D | precise timestamps |
| I²C-1 | 2 (SDA), 3 (SCL) | AS5600 + PCA9685 | 3.3 V, ~2.2 kΩ pull-ups |
| Power | 5 V / 3.3 V | sensors/hat | common ground |

Long I²C up a mast can be flaky—optionally a masthead MCU speaking RS-485/CAN can be added.

---

## Core ROS 2 Graph
/gnss/nmea ← nmea_serial_driver (UART)
/fix ← nmea_navsat_driver (NavSatFix)
/imu/data_raw ← vic3d_imu_node (gyro+accel only)
/imu/data ← imu_filter_madgwick (RPY; use_mag=false)
/yaw/deg ← heading_estimator_node (yaw corrected toward GPS COG)
/wind/apparent_angle ← as5600_wind_node
/rudder/cmd ← heading_controller_node
/sail/cmd ← sail_controller_node
/actuators/* ← pca9685_servo_node
/hard_kill ← watchdog_node