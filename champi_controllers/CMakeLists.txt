cmake_minimum_required(VERSION 3.8)
project(champi_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(champi_libraries REQUIRED)
find_package(Protobuf REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(diagnostic_updater REQUIRED)

add_executable(base_controller_node src/base_controller_node.cpp)
ament_target_dependencies(base_controller_node rclcpp geometry_msgs champi_libraries Protobuf nav_msgs tf2 tf2_ros diagnostic_updater)

add_executable(imu_controller_node src/imu_controller_node.cpp)
ament_target_dependencies(imu_controller_node rclcpp champi_libraries Protobuf diagnostic_updater sensor_msgs)

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

# Allow launch files to be found when calling them with ros2 launch
install(DIRECTORY
        launch
#        config
        # resources
        DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
        base_controller_node
        imu_controller_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
