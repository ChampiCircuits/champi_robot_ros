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
find_package(champi_libraries_cpp REQUIRED)
find_package(Protobuf REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(std_msgs REQUIRED)

# Include headers from champi_controllers' include directory
include_directories(include)

# Add the libmodbus library
find_library(MODBUS_LIBRARY modbus REQUIRED)
include_directories(${MODBUS_INCLUDE_DIRS})

add_executable(base_controller_node src/base_controller_node.cpp)
ament_target_dependencies(base_controller_node rclcpp geometry_msgs champi_libraries_cpp Protobuf nav_msgs tf2 tf2_ros diagnostic_updater)

add_executable(imu_controller_node src/imu_controller_node.cpp)
ament_target_dependencies(imu_controller_node rclcpp champi_libraries_cpp Protobuf diagnostic_updater sensor_msgs)

add_executable(actuators_controller_node src/actuators_controller_node.cpp)
ament_target_dependencies(actuators_controller_node rclcpp champi_libraries_cpp Protobuf diagnostic_updater std_msgs)

add_executable(hardware_interface_node src/hardware_interface_node.cpp)
ament_target_dependencies(hardware_interface_node rclcpp)
target_link_libraries(hardware_interface_node ${MODBUS_LIBRARY})


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
        actuators_controller_node
        hardware_interface_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
