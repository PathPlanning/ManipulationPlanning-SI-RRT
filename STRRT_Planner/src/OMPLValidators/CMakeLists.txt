cmake_minimum_required(VERSION 3.10)


project(MDP_OMPLValidator)
# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_CXX_FLAGS "-march=native -mtune=native -O3 ")
# set(CMAKE_C_FLAGS "-march=native -mtune=native -O3 ")

# set(CMAKE_BUILD_TYPE Debug)

# set(CMAKE_CXX_FLAGS "-O0 -g3 -fno-omit-frame-pointer -Wall -Wextra -fsanitize=address -fsanitize=undefined  -fsanitize=leak ")
# set(CMAKE_C_FLAGS "-O0 -g3 -fno-omit-frame-pointer -Wall -Wextra -fsanitize=address -fsanitize=undefined  -fsanitize=leak ")

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(ompl REQUIRED)
include_directories(${OMPL_INCLUDE_DIRS})

add_library(OMPLValidator STATIC "./SpaceTimeMotionValidator.cpp" "./StateValidityCheckerFunctor.cpp")
target_link_libraries(OMPLValidator PUBLIC ompl::ompl)  
target_include_directories(OMPLValidator PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)