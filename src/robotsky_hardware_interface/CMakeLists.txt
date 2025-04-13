cmake_minimum_required(VERSION 3.8)
project(robotsky_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(spdlog REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  src
)

add_library(robotsky_motor_lib
  src/motor/utils/cyber_gear_utils.cpp
  src/motor/utils/lk_motor_utils.cpp
  src/motor/utils/dm_motor_utils.cpp
  src/can/can_driver.cpp

  # src/motor/motor_control.cpp
)
target_link_libraries(robotsky_motor_lib
  spdlog::spdlog
  Eigen3::Eigen
)
ament_target_dependencies(robotsky_motor_lib
  rclcpp

  # std_msgs
  # sensor_msgs
  # srobot_msg
  # serial
)

add_executable(motor_control_test
  src/motor_control_test.cpp
)
target_link_libraries(motor_control_test
  ${catkin_LIBRARIES}
  spdlog::spdlog
  Eigen3::Eigen
  robotsky_motor_lib
)

install(TARGETS
  motor_control_test
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
