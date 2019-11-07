##############################################
#          Library Creation                  #
##############################################
set(LIBRARY_NAME scrimmage-core)
set(LIB_MAJOR 0)
set(LIB_MINOR 0)
set(LIB_RELEASE 1)

set(SRCS
    autonomy/Autonomy.cpp
    common/ColorMaps.cpp common/FileSearch.cpp common/ID.cpp common/PID.cpp
    common/Random.cpp common/RTree.cpp common/Timer.cpp common/Utilities.cpp
    common/CSV.cpp
    common/VariableIO.cpp
    common/Battery.cpp
    common/Shape.cpp
    entity/Contact.cpp entity/Entity.cpp entity/External.cpp
    entity/EntityPlugin.cpp
    log/FrameUpdateClient.cpp log/Log.cpp
    math/Angles.cpp math/Quaternion.cpp math/State.cpp
    math/StateWithCovariance.cpp
    metrics/Metrics.cpp
    network/Interface.cpp network/ScrimmageServiceImpl.cpp
    parse/ConfigParse.cpp parse/MissionParse.cpp parse/ParseUtils.cpp
    plugin_manager/MotionModel.cpp plugin_manager/Plugin.cpp
    plugin_manager/PluginManager.cpp
    proto_conversions/ProtoConversions.cpp
    pubsub/MessageBase.cpp pubsub/SubscriberBase.cpp pubsub/Network.cpp
    pubsub/NetworkDevice.cpp pubsub/Publisher.cpp pubsub/PubSub.cpp
    sensor/Sensor.cpp
    simcontrol/SimControl.cpp
    simcontrol/SimUtils.cpp
    common/DelayedTask.cpp
    common/ExponentialFilter.cpp
    common/ParameterServer.cpp
    common/GlobalService.cpp
    common/Waypoint.cpp
    common/WaypointListProcessor.cpp
)

add_library(${LIBRARY_NAME} SHARED
  ${SRCS}
)

add_dependencies(${LIBRARY_NAME}
  scrimmage-msgs scrimmage-proto-gen
)

if (GRPC_FOUND)
  target_compile_definitions(${LIBRARY_NAME} PUBLIC -DENABLE_GRPC=1)
else()
  target_compile_definitions(${LIBRARY_NAME} PUBLIC -DENABLE_GRPC=0)
endif()

target_link_libraries(${LIBRARY_NAME}
  PUBLIC
    scrimmage-protos
    scrimmage-msgs
    ${GeographicLib_LIBRARIES}
    ${CMAKE_DL_LIBS}
  PRIVATE
    Boost::boost
    Boost::filesystem
    Boost::program_options
    Boost::date_time
    Boost::graph
    Boost::thread
)

if(ENABLE_PYTHON_BINDINGS)
  # compile definitions need to be public
  # but the rest can be private
  get_target_property(python_def scrimmage-python INTERFACE_COMPILE_DEFINITIONS)
  target_compile_definitions(${LIBRARY_NAME} PUBLIC ${python_def})
  target_link_libraries(
    ${LIBRARY_NAME} PRIVATE scrimmage-python)
endif()

set(_soversion ${LIB_MAJOR}.${LIB_MINOR}.${LIB_RELEASE})

set_target_properties(
  ${LIBRARY_NAME}
  PROPERTIES
  SOVERSION ${LIB_MAJOR}
  VERSION ${_soversion}
)

target_include_directories(${LIBRARY_NAME}
  PUBLIC
    ${EIGEN3_INCLUDE_DIR}
    ${GeographicLib_INCLUDE_DIRS}
    "$<INSTALL_INTERFACE:include>"
    "$<BUILD_INTERFACE:${SCRIMMAGE_INC_DIR}>"
)

install(TARGETS ${LIBRARY_NAME}
  # IMPORTANT: Add the library to the "export-set"
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)

##################################################
# Recurse into protos and plugins
##################################################
add_subdirectory(proto)
add_subdirectory(plugins)

# Ensure every plugin depends on scrimmage, scrimmage-msgs, and scrimmage-proto
foreach(plugin ${PROJECT_PLUGINS})
  add_dependencies(${plugin} scrimmage-core scrimmage-msgs scrimmage-proto-gen)
endforeach()

# Build visualizer is VTK is found
if(NOT EXTERNAL AND ${VTK_FOUND})
  add_subdirectory(viewer)
endif()

# Push up the SCRIMMAGE_PLUGINS variable
set(PROJECT_PLUGINS ${PROJECT_PLUGINS} PARENT_SCOPE)
