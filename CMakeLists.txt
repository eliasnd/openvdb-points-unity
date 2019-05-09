cmake_minimum_required(VERSION 2.8.12)
PROJECT(openvdb-points-unity)
include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

add_library(openvdb-points-unity SHARED ${PROJECT_SOURCE_DIR}/src/openvdb-points-unity.cpp)
target_link_libraries(openvdb-points-unity openvdb tbb Half)

if(UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -lstdc++ -lm -DOPENVDB_3_ABI_COMPATIBLE")
endif()