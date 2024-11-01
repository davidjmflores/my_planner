# david_planner/CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 20)
project(david_planner)

add_library(david_planner_lib STATIC lib/planning/libdavid_planning_three.a)

set_target_properties(david_planner_lib PROPERTIES
    LINKER_LANGUAGE CXX
)

target_include_directories(david_planner_lib PUBLIC 
include
include/external/eigen
include/external/assert/include
include/external/sophus_lie)
