cmake_minimum_required(VERSION 3.8)
project(robotsky_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial REQUIRED)

find_package(spdlog REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(src)

add_library(robotsky_imu_lib
  src/imu/ahrs_driver.cpp
  src/imu/crc_table.cpp
)
target_link_libraries(robotsky_imu_lib
  spdlog::spdlog
  Eigen3::Eigen
)
ament_target_dependencies(robotsky_imu_lib
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  serial
)

add_executable(imu_node
  src/imu_node.cpp
)
target_link_libraries(imu_node
  spdlog::spdlog
  Eigen3::Eigen
  robotsky_imu_lib
)
ament_target_dependencies(imu_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  serial
)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  imu_node
  DESTINATION lib/${PROJECT_NAME}
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
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
