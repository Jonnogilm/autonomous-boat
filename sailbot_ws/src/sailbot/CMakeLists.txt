cmake_minimum_required(VERSION 3.8)
project(sailbot)

# ---------------------------------------------------------
# Default C++ standard
# ---------------------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ---------------------------------------------------------
# Dependencies
# ---------------------------------------------------------
find_package(ament_cmake REQUIRED)


find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)



# ---------------------------------------------------------
# Includes
# ---------------------------------------------------------
include_directories(include)

# ---------------------------------------------------------
# BUILD LIBRARIES
# ---------------------------------------------------------

# ---- ACTUATORS ----
add_library(sailbot_actuators
  src/actuators/pca9685.cpp
  src/actuators/rudder_servo.cpp
  src/actuators/winch_servo.cpp
)
ament_target_dependencies(sailbot_actuators rclcpp std_msgs)
target_link_libraries(sailbot_actuators)

# ---- SENSORS ----
add_library(sailbot_sensors
  src/sensors/wind_sensor.cpp
  src/sensors/bno055_i2c.cpp
)
ament_target_dependencies(sailbot_sensors rclcpp sensor_msgs)
target_link_libraries(sailbot_sensors)

find_library(LGPIO_LIB lgpio)
if(NOT LGPIO_LIB)
  message(FATAL_ERROR "Could not find lgpio library (liblgpio). Install it or ensure it is in the linker path.")
endif()
target_link_libraries(sailbot_sensors ${LGPIO_LIB})


# ---- CONTROL ----
add_library(sailbot_control
  src/control/rudder_controller.cpp
  src/control/sheet_controller.cpp
)
ament_target_dependencies(sailbot_control rclcpp std_msgs)
target_link_libraries(sailbot_control)



# ---------------------------------------------------------
# EXPORT LIBRARIES
# ---------------------------------------------------------
ament_export_include_directories(include)

ament_export_libraries(
  sailbot_actuators
  sailbot_sensors
  sailbot_control
)
# ---------------------------------------------------------
# EXECUTABLE NODES
# ---------------------------------------------------------

# --- BNO055 IMU ---
add_executable(bno055_i2c_node src/bno055_i2c_node.cpp)
ament_target_dependencies(bno055_i2c_node rclcpp std_msgs sensor_msgs)
target_link_libraries(bno055_i2c_node sailbot_sensors)

add_executable(wind_sensor_node src/wind_sensor_node.cpp)
ament_target_dependencies(wind_sensor_node rclcpp std_msgs)
target_link_libraries(wind_sensor_node sailbot_sensors)

# --- ACTUATORS ---
add_executable(actuators_node src/actuators_node.cpp)
ament_target_dependencies(actuators_node rclcpp std_msgs)
target_link_libraries(actuators_node sailbot_actuators)

# --- CONTROL ---
add_executable(control_node src/control_node.cpp)
ament_target_dependencies(control_node rclcpp std_msgs)
target_link_libraries(control_node sailbot_control sailbot_actuators)

# --- MISSION ---
add_executable(mission_node src/mission_node.cpp)
ament_target_dependencies(mission_node rclcpp std_msgs geometry_msgs)


# ---------------------------------------------------------
# INSTALL
# ---------------------------------------------------------

install(
  TARGETS
    sailbot_actuators
    sailbot_sensors
    sailbot_control

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  TARGETS
    bno055_i2c_node
    actuators_node
    control_node
    mission_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(TARGETS wind_sensor_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/data_logger.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

# rm -rf build install log
# colcon build --event-handlers console_direct+
# source install/setup.bash
# ros2 launch sailbot sailbot_launch.py