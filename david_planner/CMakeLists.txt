# david_planner/CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(david_planner)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/external)

add_library(david_planner_lib STATIC lib/planning/libdavid_planning_three.a)

set_target_properties(david_planner_lib PROPERTIES
    LINKER_LANGUAGE CXX
)

target_include_directories(david_planner_lib PUBLIC include)
