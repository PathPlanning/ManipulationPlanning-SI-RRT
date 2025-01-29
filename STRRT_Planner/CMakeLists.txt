cmake_minimum_required(VERSION 3.10)

project(STRRT_Planner)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-march=native -mtune=native -O3 ")
set(CMAKE_C_FLAGS "-march=native -mtune=native -O3 ")

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find OMPL
include_directories("../ompl/src")
find_package(ompl REQUIRED)
include_directories(${OMPL_INCLUDE_DIRS})

# Find RapidJSON
find_package(RapidJSON REQUIRED)
# Include directories
include_directories(include)
add_subdirectory("./src/config_read_writer")

add_subdirectory("./src/CollisionManager")

add_subdirectory("./src/OMPLValidators")

# Add executable
add_executable(STRRT_Planner "src/main.cpp")

add_executable(check_scene "src/util_executables/check_scene.cpp")

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${Eigen3_INCLUDE_DIRS})

# Link libraries
target_link_libraries(STRRT_Planner PUBLIC urdf orocos-kdl OMPLValidator Eigen3::Eigen config_read_writer CollisionManager ompl::ompl  pthread )

target_link_libraries(check_scene PUBLIC urdf orocos-kdl Eigen3::Eigen config_read_writer CollisionManager  pthread )
