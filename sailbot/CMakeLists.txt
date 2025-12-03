# CMakeLists.txt (relevant bits)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(sailbot_actuators
  src/actuators/pca9685.cpp
  src/actuators/rudder_servo.cpp
  src/actuators/winch_servo.cpp)
target_include_directories(sailbot_actuators PUBLIC include)
target_compile_features(sailbot_actuators PUBLIC cxx_std_17)

add_executable(actuators_node src/actuators_node.cpp)
target_link_libraries(actuators_node PRIVATE sailbot_actuators)
ament_target_dependencies(actuators_node rclcpp std_msgs)

install(TARGETS actuators_node sailbot_actuators
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
