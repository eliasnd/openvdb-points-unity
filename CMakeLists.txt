cmake_minimum_required(VERSION 2.8.12)
PROJECT(openvdb-points-unity)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
list(APPEND CMAKE_MODULE_PATH ${OPENVDB_MODULE_DIR})

# SET UP RPATH HANDLING
if (APPLE)
    set(CMAKE_INSTALL_RPATH "@loader_path")
else()    
    set(CMAKE_INSTALL_RPATH "$ORIGIN")
endif()
set(CMAKE_BUILD_WITH_INSTALL_RPATH ON)

# FIND ALL REQUIRED LIBRARIES
find_package(OpenVDB REQUIRED)
include_directories(${OPENVDB_VSCODE_INCLUDE_DIR})
add_library(openvdb-points-unity SHARED ${PROJECT_SOURCE_DIR}/src/openvdb-points-unity.cpp)
target_link_libraries(openvdb-points-unity OpenVDB::openvdb)
set(OPENVDB_POINTS_UNITY_INCLUDE_FILES ${PROJECT_SOURCE_DIR}/src/openvdb-points-unity.h)
if(UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -lstdc++ -lm -DOPENVDB_ABI_VERSION_NUMBER=7")
endif()
install(TARGETS openvdb-points-unity DESTINATION lib)
install(FILES ${OPENVDB_POINTS_UNITY_INCLUDE_FILES} DESTINATION include/openvdb-points-unity)
