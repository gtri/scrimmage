cmake_minimum_required(VERSION 3.10)
project(c2overlay)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(scrimmage REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gRPC CONFIG REQUIRED)

set(PROJECT_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(PROJECT_PLUGIN_LIBS_DIR ${CMAKE_CURRENT_BINARY_DIR}/plugin_libs)

include_directories(${PROJECT_INC_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR}/msgs)

add_subdirectory(msgs)
add_subdirectory(src/plugins/interaction/TopicTap)

enable_testing()
add_subdirectory(test)

include(GenerateSetEnv)
GenerateSetEnv(
  SETUP_LOCAL_CONFIG_DIR OFF
  SETENV_IN_FILE ${SCRIMMAGE_CMAKE_MODULES}/setenv.in
  MISSION_PATH ${PROJECT_SOURCE_DIR}/missions
  PLUGIN_PATH ${PROJECT_PLUGIN_LIBS_DIR}
              ${PROJECT_SOURCE_DIR}/include/c2overlay/plugins
)

install(DIRECTORY ${PROJECT_PLUGIN_LIBS_DIR}/
  DESTINATION lib/c2overlay/plugin_libs
  FILES_MATCHING PATTERN "*.so")
install(DIRECTORY ${PROJECT_SOURCE_DIR}/missions/
  DESTINATION share/c2overlay/missions)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/c2overlay/plugins/
  DESTINATION etc/c2overlay/plugins
  FILES_MATCHING PATTERN "*.xml")
