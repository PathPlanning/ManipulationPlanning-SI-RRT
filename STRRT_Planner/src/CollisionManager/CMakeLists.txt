cmake_minimum_required(VERSION 3.10)

project(MDP_CollisionManager)
# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_CXX_FLAGS "-march=native -mtune=native -O3 -no-pie ")
# set(CMAKE_C_FLAGS "-march=native -mtune=native -O3 -no-pie ")
# set(CMAKE_BUILD_TYPE Debug)

# set(CMAKE_CXX_FLAGS "-O0 -g3 -fno-omit-frame-pointer -Wall -Wextra -fsanitize=address -fsanitize=undefined  -fsanitize=leak ")
# set(CMAKE_C_FLAGS "-O0 -g3 -fno-omit-frame-pointer -Wall -Wextra -fsanitize=address -fsanitize=undefined  -fsanitize=leak ")
# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
# find_package(coal REQUIRED)

find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)



add_library(CollisionManager STATIC "./ObjectObstacleFCL.cpp" "./CubeObstacleFCL.cpp" "./SphereObstacleFCL.cpp" "./RobotObstacleFCL.cpp"  "./CollisionManager.cpp")
target_link_libraries(CollisionManager PUBLIC urdf orocos-kdl kdl_parser coal )  
target_include_directories(CollisionManager PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../../include)