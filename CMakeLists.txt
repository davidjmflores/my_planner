cmake_minimum_required(VERSION 3.8)
project(my_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
add_subdirectory(david_planner)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Add executable
add_executable(robot_controller src/robot_controller_with_tags.cpp)
target_link_libraries(robot_controller david_planner_lib)

# Link dependencies
ament_target_dependencies(robot_controller rclcpp geometry_msgs apriltag_msgs)

# Include directories
target_include_directories(robot_controller PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/david_planner/include>
    $<INSTALL_INTERFACE:include>
)

# Install targets
install(TARGETS
  robot_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
