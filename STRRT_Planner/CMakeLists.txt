cmake_minimum_required(VERSION 3.10)

project(STRRT_Planner)

if(DEFINED CMAKE_BUILD_TYPE)
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        set(CMAKE_CXX_FLAGS "-march=native -mtune=native -O3 -no-pie -fno-pie -flto=auto ")
        set(CMAKE_C_FLAGS "-march=native -mtune=native -O3 -no-pie -fno-pie -flto=auto ")
    elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
        set(CMAKE_CXX_FLAGS "-O0 -g3 -fno-omit-frame-pointer -Wall -Wextra -fsanitize=address -fsanitize=undefined -fno-sanitize-recover=all -fsanitize=float-divide-by-zero -fsanitize=float-cast-overflow -fno-sanitize=null -fno-sanitize=alignment  -fsanitize=leak ")
        set(CMAKE_C_FLAGS "-O0 -g3 -fno-omit-frame-pointer -Wall -Wextra -fsanitize=address -fsanitize=undefined -fno-sanitize-recover=all -fsanitize=float-divide-by-zero -fsanitize=float-cast-overflow -fno-sanitize=null -fno-sanitize=alignment  -fsanitize=leak ")
    endif()
else()
    set(CMAKE_BUILD_TYPE Release)
        set(CMAKE_CXX_FLAGS "-march=native -mtune=native -O3 -no-pie -fno-pie -flto=auto ")
        set(CMAKE_C_FLAGS "-march=native -mtune=native -O3 -no-pie -fno-pie -flto=auto ")
endif()



message(STATUS "CMAKE_BUILD_TYPE is set to '${CMAKE_BUILD_TYPE}'")
message(STATUS "CMAKE_CXX_FLAGS is set to '${CMAKE_CXX_FLAGS}'")
message(STATUS "CMAKE_C_FLAGS is set to '${CMAKE_C_FLAGS}'")


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
target_link_libraries(STRRT_Planner PUBLIC urdf orocos-kdl OMPLValidator Eigen3::Eigen config_read_writer CollisionManager ompl  pthread )

target_link_libraries(check_scene PUBLIC urdf orocos-kdl Eigen3::Eigen config_read_writer CollisionManager  pthread )
