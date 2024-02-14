# Inspiration: https://www.theconstructsim.com/how-to-create-a-ros2-c-library/

cmake_minimum_required(VERSION 3.8)
project(champi_libraries)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


# ================== COMPILE LIBRARIES =======================

# let the compiler search for headers in the include folder
include_directories(include)
 
# define a library target 
file(GLOB champi_can_SRC CONFIGURE_DEPENDS "src/champi_can/*.cc" "src/champi_can/*.c")
add_library(champi_can ${champi_can_SRC})
 
# this line to exports the library
ament_export_targets(champi_can HAS_LIBRARY_TARGET)
 
# install the include/(library) directory to the install/include/my_cpp_library
install(
  DIRECTORY include/champi_can
  DESTINATION include
)
 
install(
  TARGETS champi_can
  EXPORT champi_can
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# ================ END COMPILE LIBRARIES ========================

# ================ COMPILE EXECUTABLES ========================

add_executable(test_can_node src/test_can_node.cpp)
ament_target_dependencies(test_can_node)

install(TARGETS
test_can_node
  DESTINATION lib/${PROJECT_NAME})

# ================ END COMPILE EXECUTABLES ========================

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
