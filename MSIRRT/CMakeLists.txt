cmake_minimum_required(VERSION 3.10)

project(MSIRRT_Planner_Planner)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-march=native -mtune=native -O3 -no-pie")
set(CMAKE_C_FLAGS "-march=native -mtune=native -O3 -no-pie")

# set(CMAKE_BUILD_TYPE Debug)

# set(CMAKE_CXX_FLAGS "-O0 -g ")
# set(CMAKE_C_FLAGS "-O0 -g ")


# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find RapidJSON
find_package(RapidJSON REQUIRED)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${Eigen3_INCLUDE_DIRS})
find_package(RapidJSON REQUIRED)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../rapidjson/)

# Include directories
include_directories(include)

add_subdirectory("../STRRT_Planner/src/config_read_writer" ./config_read_writer)

add_subdirectory("../STRRT_Planner/src/CollisionManager" ./CollisionManager)

# Add executable
add_library(MSIRRT_Planner_lib STATIC  "./src/PlannerConnect.cpp" "./src/Tree.cpp" "./src/Vertex.cpp")
target_link_libraries(MSIRRT_Planner_lib PUBLIC config_read_writer CollisionManager)  
target_include_directories(MSIRRT_Planner_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../../include)

add_executable(MSIRRT_Planner "src/main.cpp")
add_executable(bench_broadphase "src/benchmark_broadphase.cpp")
add_executable(get_safe_intevals_by_path "src/get_safe_intevals_by_path.cpp")


# Link libraries
target_link_libraries(MSIRRT_Planner PUBLIC MSIRRT_Planner_lib urdf orocos-kdl Eigen3::Eigen config_read_writer CollisionManager   pthread )
target_link_libraries(bench_broadphase PUBLIC MSIRRT_Planner_lib urdf orocos-kdl Eigen3::Eigen config_read_writer CollisionManager   pthread )
target_link_libraries(get_safe_intevals_by_path PUBLIC RapidJSON MSIRRT_Planner_lib urdf orocos-kdl Eigen3::Eigen CollisionManager config_read_writer   pthread )

