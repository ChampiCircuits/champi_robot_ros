cmake_minimum_required(VERSION 3.8)
project(champi_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS})

add_executable(absolute_loc_node src/absolute_loc_node.cpp)
ament_target_dependencies(absolute_loc_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  pcl_conversions
)

# add_executable(icp_pcl src/icp_pcl.cpp)
# target_link_libraries(icp_pcl ${PCL_LIBRARIES})
# ament_target_dependencies(icp_pcl rclcpp sensor_msgs geometry_msgs pcl_conversions visualization_msgs)

# Allow lauch files to be found when calling them with ros2 launch
install(DIRECTORY
  launch
  # config
  resources
  DESTINATION share/${PROJECT_NAME}
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

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
        scripts/path_planner_node.py
        scripts/pose_controller_node.py
        scripts/robot_stopped_detector_node.py
        scripts/costmap_updater_node.py
        scripts/path_planner_ui_node.py
        scripts/absolute_loc_node_py.py

        DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
        absolute_loc_node
        DESTINATION lib/${PROJECT_NAME}
)


ament_package()
