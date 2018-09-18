set (APP_NAME scrimmage-bin)

file (GLOB SRCS *.cpp)
file (GLOB HDRS *.h)

#
if(APPLE)
  set(MACOSX_BUNDLE_ICON_FILE scrimmage.icns)
  set(appIcon ${PROJECT_SOURCE_DIR}/data/icons/${MACOSX_BUNDLE_ICON_FILE})
  set_source_files_properties(${appIcon} PROPERTIES
    MACOSX_PACKAGE_LOCATION "Resources")
  add_executable(${APP_NAME} MACOSX_BUNDLE ${appIcon} ${SRCS})
else()
  add_executable(${APP_NAME} ${SRCS})
endif()


add_dependencies(${APP_NAME} scrimmage-protos)

target_link_libraries(${APP_NAME}
  ${JSBSIM_LIBRARIES}
  scrimmage-core
  scrimmage-boost
  ${SWARM_SIM_LIBS}
  dl
  )
if (ENABLE_PYTHON_BINDINGS)
  target_link_libraries(${APP_NAME} scrimmage-python)
endif()


if (NOT EXTERNAL AND ${VTK_FOUND})
  target_link_libraries(${APP_NAME}
    scrimmage-viewer
    )
endif()

if (NOT INSTALL_LINK)
  install(TARGETS ${APP_NAME} DESTINATION bin)
endif()

set_target_properties(
  ${APP_NAME}
  PROPERTIES
  OUTPUT_NAME scrimmage
)

export(TARGETS ${APP_NAME} FILE scrimmage-exports.cmake)
# Link executable to other libraries
#target_link_libraries(${APP_NAME})

##--------------------------------------------------------
## Library Creation
##--------------------------------------------------------
#SET (LIBRARY_NAME BHV_Colregs)
#SET (LIB_MAJOR 0)
#SET (LIB_MINOR 0)
#SET (LIB_RELEASE 1)
#
#file( GLOB SRCS *.cpp)
#file( GLOB HDRS *.h)
#
#ADD_LIBRARY(${LIBRARY_NAME} SHARED
#  ${SRCS}
#  )
#
#TARGET_LINK_LIBRARIES(${LIBRARY_NAME}
#  OTHERLIBS
#  )
#
#SET (_soversion ${LIB_MAJOR}.${LIB_MINOR}.${LIB_RELEASE})
#
#set_target_properties(${LIBRARY_NAME} PROPERTIES SOVERSION ${LIB_MAJOR} VERSION ${_soversion})
#
#INSTALL(TARGETS ${LIBRARY_NAME}
#  LIBRARY DESTINATION lib
#  ARCHIVE DESTINATION lib
#)
