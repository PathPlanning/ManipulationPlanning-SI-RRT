cmake_minimum_required(VERSION 3.10)

project(MDP_config_read_writer)
if(DEFINED CMAKE_BUILD_TYPE)
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        set(CMAKE_CXX_FLAGS "-march=native -mtune=native -O3 -no-pie -fno-pie -flto -fomit-frame-pointer")
        set(CMAKE_C_FLAGS "-march=native -mtune=native -O3 -no-pie -fno-pie -flto -fomit-frame-pointer")
    elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
        set(CMAKE_CXX_FLAGS "-O0 -g3 -fno-omit-frame-pointer -Wall -Wextra -fsanitize=address -fsanitize=undefined  -fsanitize=leak ")
        set(CMAKE_C_FLAGS "-O0 -g3 -fno-omit-frame-pointer -Wall -Wextra -fsanitize=address -fsanitize=undefined  -fsanitize=leak ")
    endif()
else()
    set(CMAKE_BUILD_TYPE Release)
    set(CMAKE_CXX_FLAGS "-march=native -mtune=native -O3 -no-pie -fno-pie -flto -fomit-frame-pointer")
    set(CMAKE_C_FLAGS "-march=native -mtune=native -O3 -no-pie -fno-pie -flto -fomit-frame-pointer")
endif()


message(STATUS "CMAKE_BUILD_TYPE is set to '${CMAKE_BUILD_TYPE}'")
message(STATUS "CMAKE_CXX_FLAGS is set to '${CMAKE_CXX_FLAGS}'")
message(STATUS "CMAKE_C_FLAGS is set to '${CMAKE_C_FLAGS}'")

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
find_package(RapidJSON REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../../rapidjson/)

add_library(config_read_writer STATIC "./config_read.cpp" "./ObstacleJsonInfo.cpp" "./CubeObstacleJsonInfo.cpp" "./SphereObstacleJsonInfo.cpp" "./RobotObstacleJsonInfo.cpp" "./STRRTConfigReader.cpp" "./ResultsWriter.cpp")
target_link_libraries(config_read_writer  PUBLIC RapidJSON Eigen3::Eigen)  
target_include_directories(config_read_writer PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../../include )