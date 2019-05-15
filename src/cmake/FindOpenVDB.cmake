# find_package(OpenVDB [version] [REQUIRED] openvdb (libopenvdb))

if(APPLE)
    set(OPENVDB_SHARED_LIB_SUFFIX ".dylib")
elseif(UNIX AND NOT APPLE)
    set(OPENVDB_SHARED_LIB_SUFFIX ".so")
else()
    set(OPENVDB_SHARED_LIB_SUFFIX ".dll")
endif()

if(OPENVDB_ROOT)
    set(OPENVDB_LIB_DIR ${OPENVDB_ROOT}/lib)
    set(OPENVDB_INCLUDE_DIR ${OPENVDB_ROOT/include})
    find_library(OPENVDB_LIB NO_DEFAULT_PATH NAMES openvdb HINTS ${OPENVDB_LIB_DIR})
endif()

if(NOT OPENVDB_ROOT)
    message(Could not find OpenVDB lib in "${OPENVDB_LIB_DIR}")
endif()