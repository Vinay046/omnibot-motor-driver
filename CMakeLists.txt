cmake_minimum_required(VERSION 3.5)
project(omnibot_motor_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(omnibot_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

# Build
add_executable(motor_driver src/motor_driver.cpp)
ament_target_dependencies(motor_driver
  omnibot_msgs
  dynamixel_sdk
  std_msgs
  rclcpp
  rclcpp_lifecycle
  diagnostic_msgs
  sensor_msgs
  std_srvs
)

# Install
install(TARGETS
  motor_driver
  DESTINATION lib/${PROJECT_NAME}
)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
