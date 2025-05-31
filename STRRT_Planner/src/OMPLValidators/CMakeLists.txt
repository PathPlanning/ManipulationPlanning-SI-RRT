cmake_minimum_required(VERSION 3.10)


project(MDP_OMPLValidator)
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

find_package(ompl REQUIRED)
include_directories(${OMPL_INCLUDE_DIRS})

add_library(OMPLValidator STATIC "./SpaceTimeMotionValidator.cpp" "./StateValidityCheckerFunctor.cpp")
target_link_libraries(OMPLValidator PUBLIC ompl)  
target_include_directories(OMPLValidator PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)