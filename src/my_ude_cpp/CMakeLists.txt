cmake_minimum_required(VERSION 3.8)
project(my_ude_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ude_interface REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# add executebles
add_executable(encoder src/encoder.cpp)
ament_target_dependencies(encoder rclcpp std_msgs)

add_executable(speedometer src/speedometer.cpp)
ament_target_dependencies(speedometer rclcpp std_msgs)

add_executable(check_server src/check_server.cpp)
ament_target_dependencies(check_server rclcpp ude_interface)

add_executable(check_client src/check_client.cpp)
ament_target_dependencies(check_client rclcpp ude_interface)

add_executable(turn_camera_server src/turn_camera_server.cpp)
ament_target_dependencies(turn_camera_server rclcpp ude_interface std_msgs OpenCV cv_bridge)

add_executable(turn_camera_client src/turn_camera_client.cpp)
ament_target_dependencies(turn_camera_client rclcpp ude_interface OpenCV cv_bridge)

add_executable(navigation_server src/navigation_server.cpp)
ament_target_dependencies(navigation_server rclcpp ude_interface rclcpp_action geometry_msgs)

add_executable(navigation_client src/navigation_client.cpp)
ament_target_dependencies(navigation_client rclcpp ude_interface rclcpp_action geometry_msgs example_interfaces)

# Install Targets
install(TARGETS
  encoder
  speedometer
  check_server
  check_client
  turn_camera_server
  turn_camera_client
  navigation_server
  navigation_client
DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
