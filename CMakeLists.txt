cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 20)
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
add_executable(motion_controller_node src/motion_controller_node.cpp)
target_link_libraries(motion_controller_node david_planner_lib)
add_executable(tag_detector_node src/tag_detector_node.cpp)
target_link_libraries(tag_detector_node david_planner_lib)

# Link dependencies
ament_target_dependencies(motion_controller_node rclcpp geometry_msgs)
ament_target_dependencies(tag_detector_node rclcpp apriltag_msgs)

# Include directories
target_include_directories(motion_controller_node PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/david_planner/include>
    $<INSTALL_INTERFACE:include>
)
target_include_directories(tag_detector_node PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/david_planner/include>
    $<INSTALL_INTERFACE:include>
)

# Install targets
install(TARGETS
  motion_controller_node
  tag_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
