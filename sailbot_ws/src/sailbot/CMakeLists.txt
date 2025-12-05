cmake_minimum_required(VERSION 3.8)
project(sailbot)

# ------------------------------------------
# Default C++ standard
# ------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ------------------------------------------
# Dependencies
# ------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(rosidl_default_generators REQUIRED)

# ------------------------------------------
# Build Messages
# ------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NavState.msg"
  "msg/MissionState.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# ------------------------------------------
# Include
# ------------------------------------------
include_directories(include)

# ------------------------------------------
# BUILD LIBRARIES
# ------------------------------------------

add_library(sailbot_actuators
  src/actuators/pca9685.cpp
  src/actuators/rudder_servo.cpp
  src/actuators/winch_servo.cpp
)
ament_target_dependencies(sailbot_actuators rclcpp std_msgs)

add_library(sailbot_sensors
  src/sensors/wind_sensor.cpp
  src/sensors/imu_sensor.cpp
  src/sensors/gnss_sensor.cpp
)
ament_target_dependencies(sailbot_sensors rclcpp sensor_msgs)

add_library(sailbot_estimation
  src/estimation/nav_state_estimator.cpp
  src/estimation/true_wind_estimator.cpp
  src/estimation/velocity_filter.cpp
)
ament_target_dependencies(sailbot_estimation rclcpp geometry_msgs)

add_library(sailbot_guidance
  src/guidance/waypoint_nav.cpp
  src/guidance/wind_model.cpp
  src/guidance/tack_planner.cpp
)
ament_target_dependencies(sailbot_guidance rclcpp geometry_msgs)

add_library(sailbot_control
  src/control/rudder_controller.cpp
  src/control/sheet_controller.cpp
)
ament_target_dependencies(sailbot_control rclcpp std_msgs)

add_library(sailbot_mission
  src/mission/fsm.cpp
  src/mission/mission_manager.cpp
)
ament_target_dependencies(sailbot_mission rclcpp std_msgs geometry_msgs)

add_library(sailbot_utils
  src/utils/angles.cpp
  src/utils/filters.cpp
)
ament_target_dependencies(sailbot_utils rclcpp)

# ------------------------------------------
# EXECUTABLE NODES
# ------------------------------------------

add_executable(teseo_serial_node src/gnss/teseo_serial_node.cpp)
ament_target_dependencies(teseo_serial_node rclcpp std_msgs)

add_executable(actuators_node src/actuators_node.cpp)
ament_target_dependencies(actuators_node rclcpp std_msgs)
target_link_libraries(actuators_node sailbot_actuators)

add_executable(nav_state_estimator_node src/estimation/nav_state_estimator_node.cpp)
ament_target_dependencies(nav_state_estimator_node
  rclcpp
  std_msgs
  sensor_msgs
  builtin_interfaces
)

target_link_libraries(nav_state_estimator_node
  sailbot_sensors
  sailbot_estimation
)

add_executable(sailbot_main src/sailbot_main.cpp)
ament_target_dependencies(sailbot_main rclcpp geometry_msgs std_msgs)
target_link_libraries(
  sailbot_main
  sailbot_actuators
  sailbot_sensors
  sailbot_estimation
  sailbot_guidance
  sailbot_control
  sailbot_mission
  sailbot_utils
)

# ------------------------------------------
# INSTALL
# ------------------------------------------
install(
  TARGETS
    sailbot_actuators
    sailbot_sensors
    sailbot_estimation
    sailbot_guidance
    sailbot_control
    sailbot_mission
    sailbot_utils
    teseo_serial_node
    actuators_node
    sailbot_main
    nav_state_estimator_node
  DESTINATION lib/${PROJECT_NAME}
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

ament_package()

# To build with compile commands for clangd:
# source /opt/ros/jazzy/setup.bash
# rm -rf build install log
# colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
