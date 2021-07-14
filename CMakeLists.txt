cmake_minimum_required(VERSION 3.12)
PROJECT(openvdb-points-unity)

# list(APPEND CMAKE_MODULE_PATH ${OPENVDB_MODULE_DIR})
list(APPEND CMAKE_MODULE_PATH "~/DSL/openvdb/cmake")

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
add_library(openvdb-points-unity SHARED ${PROJECT_SOURCE_DIR}/src/openvdb-points-interface.cpp)
target_link_libraries(openvdb-points-unity OpenVDB::openvdb)
set(OPENVDB_POINTS_UNITY_INCLUDE_FILES ${PROJECT_SOURCE_DIR}/src/openvdb-points-interface.h)
if(UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++14 -lstdc++ -lm -DOPENVDB_ABI_VERSION_NUMBER=8")
endif()
install(TARGETS openvdb-points-unity DESTINATION lib)
install(FILES ${OPENVDB_POINTS_UNITY_INCLUDE_FILES} DESTINATION include/openvdb-points-unity)
set(CMAKE_BUILD_TYPE Debug)
