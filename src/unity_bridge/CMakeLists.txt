###############################################################################
# SCRIMMAGE Unity Bridge
#
# Requires:
#   - ZeroMQ C++ bindings (cppzmq): find_package(cppzmq) or pkg-config
#   - nlohmann_json: find_package(nlohmann_json) or header-only include
#
# Enable with: cmake -DBUILD_UNITY_BRIDGE=ON ..
###############################################################################

if (NOT BUILD_UNITY_BRIDGE)
    return()
endif()

###############################################################################
# Dependencies
###############################################################################

# ZeroMQ: prefer cppzmq (header-only C++ binding over libzmq)
find_package(cppzmq QUIET)
if (NOT cppzmq_FOUND)
    # Fall back to pkg-config
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(ZMQ REQUIRED libzmq)
    message(STATUS "Unity Bridge: using libzmq via pkg-config")
else()
    message(STATUS "Unity Bridge: using cppzmq ${cppzmq_VERSION}")
endif()

# nlohmann/json — header-only JSON library
find_package(nlohmann_json QUIET)
if (NOT nlohmann_json_FOUND)
    # Try to find the single-header include
    find_path(NLOHMANN_JSON_INCLUDE_DIR nlohmann/json.hpp
        HINTS /usr/include /usr/local/include)
    if (NOT NLOHMANN_JSON_INCLUDE_DIR)
        message(FATAL_ERROR
            "Unity Bridge: nlohmann/json not found. "
            "Install with: sudo apt install nlohmann-json3-dev")
    endif()
    add_library(nlohmann_json INTERFACE)
    target_include_directories(nlohmann_json INTERFACE ${NLOHMANN_JSON_INCLUDE_DIR})
    message(STATUS "Unity Bridge: found nlohmann/json at ${NLOHMANN_JSON_INCLUDE_DIR}")
endif()

###############################################################################
# Library target
###############################################################################

set(UNITY_BRIDGE_SOURCES
    UnityBridge.cpp
)

add_library(scrimmage_unity_bridge SHARED ${UNITY_BRIDGE_SOURCES})

add_dependencies(scrimmage_unity_bridge scrimmage-proto-gen scrimmage-msgs)

target_include_directories(scrimmage_unity_bridge
    PUBLIC
    "$<BUILD_INTERFACE:${SCRIMMAGE_INC_DIR}>"
    "$<INSTALL_INTERFACE:include>"
)

if (cppzmq_FOUND)
    target_link_libraries(scrimmage_unity_bridge
        PUBLIC
            cppzmq
            scrimmage-protos
    )
else()
    target_link_libraries(scrimmage_unity_bridge
        PUBLIC
            ${ZMQ_LIBRARIES}
            scrimmage-protos
    )
    target_include_directories(scrimmage_unity_bridge PUBLIC ${ZMQ_INCLUDE_DIRS})
endif()

target_link_libraries(scrimmage_unity_bridge PUBLIC nlohmann_json)

target_compile_definitions(scrimmage_unity_bridge
    PUBLIC ENABLE_UNITY_BRIDGE=1
)

set_target_properties(scrimmage_unity_bridge PROPERTIES
    VERSION   ${PROJECT_VERSION}
    SOVERSION ${SCRIMMAGE_SOVERSION}
)

###############################################################################
# Install
###############################################################################

install(TARGETS scrimmage_unity_bridge
    EXPORT  scrimmage-targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(
    DIRECTORY   ${CMAKE_SOURCE_DIR}/include/scrimmage/unity_bridge
    DESTINATION include/scrimmage
)
