cmake_minimum_required(VERSION 3.8)
project(champi_nav2)

find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(angles REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_behaviors REQUIRED)

include_directories(
        include
)

set(dependencies
        angles
        rclcpp
        rclcpp_components
        rclcpp_action
        rclcpp_components
        std_msgs
        nav2_msgs
        nav_2d_utils
        nav_2d_msgs
        nav2_util
        nav2_core
        pluginlib
        nav2_behaviors
)

# Angle Smoother plugin
add_library(angle_smoother SHARED
        src/angle_smoother.cpp
)
ament_target_dependencies(angle_smoother
        ${dependencies}
)
# Drive To Heading Omni plugin
add_library(drive_on_heading_omni SHARED
        src/drive_on_heading_omni.cpp
)
ament_target_dependencies(drive_on_heading_omni
        ${dependencies}
)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)



if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
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

install(
        TARGETS angle_smoother
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(
        TARGETS drive_on_heading_omni
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)


install(DIRECTORY include/
        DESTINATION include/
)

# Allow launch files to be found when calling them with ros2 launch
install(DIRECTORY
        launch
        config
        # ressources
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
