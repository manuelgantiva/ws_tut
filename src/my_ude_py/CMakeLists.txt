cmake_minimum_required(VERSION 3.8)
project(my_ude_py)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ude_interface REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

ament_python_install_package(scripts)

install(PROGRAMS
  scripts/encoder.py 
  scripts/speedometer.py
  scripts/check_server.py
  scripts/check_client.py
  scripts/turn_camera_server.py
  scripts/turn_camera_client.py
  scripts/navigation_server.py
  scripts/navigation_client.py

  DESTINATION lib/${PROJECT_NAME}
)


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
