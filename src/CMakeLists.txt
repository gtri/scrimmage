add_subdirectory(proto)
add_subdirectory(plugins)

if (NOT EXTERNAL AND ${VTK_FOUND})
  add_subdirectory(viewer)
endif()

##############################################
#          Library Creation                  #
##############################################
SET (LIBRARY_NAME scrimmage)
SET (LIB_MAJOR 0)
SET (LIB_MINOR 0)
SET (LIB_RELEASE 1)

set(SRCS
    autonomy/Autonomy.cpp
    common/ColorMaps.cpp common/FileSearch.cpp common/ID.cpp common/PID.cpp
    common/Random.cpp common/RTree.cpp common/Timer.cpp common/Utilities.cpp
    entity/Contact.cpp entity/Entity.cpp entity/External.cpp
    log/FrameUpdateClient.cpp log/Log.cpp
    math/Angles.cpp math/Quaternion.cpp math/State.cpp
    metrics/Metrics.cpp
    network/Interface.cpp network/ScrimmageServiceImpl.cpp
    parse/ConfigParse.cpp parse/MissionParse.cpp parse/ParseUtils.cpp
    plugin_manager/MotionModel.cpp plugin_manager/Plugin.cpp
    plugin_manager/PluginManager.cpp
    proto_conversions/ProtoConversions.cpp
    pubsub/MessageBase.cpp pubsub/Network.cpp pubsub/NetworkDevice.cpp pubsub/Publisher.cpp
    sensor/Sensor.cpp
    simcontrol/SimControl.cpp
)


ADD_LIBRARY( ${LIBRARY_NAME} SHARED
  ${SRCS}
  )

add_dependencies(${LIBRARY_NAME}
  scrimmage-proto-gen
)

target_link_libraries(
  ${LIBRARY_NAME}
  scrimmage-protos
  ${PYTHON_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  scrimmage-msgs
  ${CMAKE_DL_LIBS}
)

SET (_soversion ${LIB_MAJOR}.${LIB_MINOR}.${LIB_RELEASE})

set_target_properties(
  ${LIBRARY_NAME}
  PROPERTIES
  SOVERSION ${LIB_MAJOR}
  VERSION ${_soversion}
  )

install(TARGETS ${LIBRARY_NAME}
  # IMPORTANT: Add the library to the "export-set"
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)

# Push up the SCRIMMAGE_LIBS variable
set(PROJECT_LIBS ${PROJECT_LIBS} ${LIBRARY_NAME} PARENT_SCOPE)
set(PROJECT_PLUGINS ${PROJECT_PLUGINS} PARENT_SCOPE)
