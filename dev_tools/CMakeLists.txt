cmake_minimum_required(VERSION 3.8)
project(dev_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(champi_libraries_cpp REQUIRED)
find_package(Protobuf REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(test_diag src/test_diag.cpp)
ament_target_dependencies(test_diag rclcpp diagnostic_updater)

add_executable(fun_node src/fun_node.cpp)
ament_target_dependencies(fun_node rclcpp geometry_msgs nav_msgs sensor_msgs champi_libraries_cpp Protobuf)

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


# Allow lauch files to be found when calling them with ros2 launch
install(DIRECTORY
  launch
  # config
  # ressources
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
        scripts/holo_teleop_joy_node.py
        scripts/pub_goal_poses.py
        scripts/call_set_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install C++ executables
install(TARGETS
        test_diag
        fun_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
