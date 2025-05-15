cmake_minimum_required(VERSION 3.8)
project(champi_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


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
  config
  ressources
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
        scripts/aruco_localizer_node.py
        scripts/camera_info_publisher_node.py
        scripts/bird_view_publisher_node.py
        scripts/cv2_camera_publisher_node.py
        scripts/enemy_tracker_node.py
        scripts/platform_detection_node.py
        scripts/board_platform_detection_node.py
        DESTINATION lib/${PROJECT_NAME}
) 

# Install C++ executables
install(TARGETS
        # enemy_detection_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
