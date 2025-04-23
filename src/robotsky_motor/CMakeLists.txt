cmake_minimum_required(VERSION 3.8)
project(robotsky_motor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(robotsky_interface REQUIRED)

find_package(spdlog REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  src
)

add_library(robotsky_motor_lib
  src/motor/utils/cyber_gear_utils.cpp
  src/motor/utils/lk_motor_utils.cpp
  src/motor/utils/dm_motor_utils.cpp
  src/motor/utils/rs_motor_utils.cpp
  src/motor/control/motor_control.cpp
  src/motor/control/rs_motor_control.cpp
  src/motor/control/dm_motor_control.cpp
  src/motor/control/cyber_motor_control.cpp
  src/motor/motor_factory.cpp
  src/can/bus/can_bus_manager.cpp
  src/can/bus/dm_can_bus_manager.cpp
  src/can/bus/rs_can_bus_manager.cpp
  src/can/can_bus_factory.cpp
  src/can/can_interface.cpp
  src/can/can_utils.cpp
  src/can/old/can_driver.cpp
  src/utils/utils.cpp
  src/utils/fps_counter.cpp
  src/robot/robot.cpp
)
target_link_libraries(robotsky_motor_lib
  spdlog::spdlog
  Eigen3::Eigen
)
ament_target_dependencies(robotsky_motor_lib
  rclcpp
  robotsky_interface
)

add_executable(robot_node
  src/robot_node.cpp
)
target_link_libraries(robot_node
  ${catkin_LIBRARIES}
  spdlog::spdlog
  Eigen3::Eigen
  robotsky_motor_lib
)

add_executable(motor_control_test
  src/test/motor_control_test.cpp
)
target_link_libraries(motor_control_test
  ${catkin_LIBRARIES}
  spdlog::spdlog
  Eigen3::Eigen
  robotsky_motor_lib
)

add_executable(motor_control_test_rs
  src/test/motor_control_test_rs.cpp
)
target_link_libraries(motor_control_test_rs
  ${catkin_LIBRARIES}
  spdlog::spdlog
  Eigen3::Eigen
  robotsky_motor_lib
)

add_executable(motor_control_test_dm
  src/test/motor_control_test_dm.cpp
)
target_link_libraries(motor_control_test_dm
  ${catkin_LIBRARIES}
  spdlog::spdlog
  Eigen3::Eigen
  robotsky_motor_lib
)

add_executable(motor_zero_test_rs
  src/test/motor_zero_test_rs.cpp
)
target_link_libraries(motor_zero_test_rs
  ${catkin_LIBRARIES}
  spdlog::spdlog
  Eigen3::Eigen
  robotsky_motor_lib
)

add_executable(motor_zero_test_dm
  src/test/motor_zero_test_dm.cpp
)
target_link_libraries(motor_zero_test_dm
  ${catkin_LIBRARIES}
  spdlog::spdlog
  Eigen3::Eigen
  robotsky_motor_lib
)

install(TARGETS
  robot_node
  motor_control_test_dm
  motor_control_test_rs
  motor_control_test
  motor_zero_test_rs
  motor_zero_test_dm
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
