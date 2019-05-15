cmake_minimum_required(VERSION 2.8.12)
PROJECT(openvdb-points-unity)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/src/cmake)
# FIND ALL REQUIRED LIBRARIES
find_package(OpenVDB REQUIRED)
add_library(openvdb-points-unity SHARED ${PROJECT_SOURCE_DIR}/src/openvdb-points-unity.cpp)
target_link_libraries(openvdb-points-unity ${OPENVDB_LIB})

if(UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -lstdc++ -lm -DOPENVDB_3_ABI_COMPATIBLE")
endif()