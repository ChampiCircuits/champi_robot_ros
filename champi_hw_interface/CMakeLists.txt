cmake_minimum_required(VERSION 3.8)
project(champi_hw_interface)

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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
#find_package(diagnostic_updater REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Create and configure champi_hw_interface library
include_directories(include)
add_library(champi_hw_interface SHARED src/ModbusRegister.cpp)
target_compile_definitions(champi_hw_interface PUBLIC -DMODBUS_MASTER)

# Add the libmodbus library
find_library(MODBUS_LIBRARY modbus REQUIRED)
include_directories(${MODBUS_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PUBLIC udev)

add_executable(hardware_interface_node src/hw_interface_node.cpp src/hw_core.cpp src/hw_setup.cpp src/hw_events.cpp)
ament_target_dependencies(hardware_interface_node rclcpp geometry_msgs nav_msgs tf2 champi_libraries_cpp tf2_geometry_msgs)
target_link_libraries(hardware_interface_node champi_hw_interface ${MODBUS_LIBRARY})


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
        hardware_interface_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
