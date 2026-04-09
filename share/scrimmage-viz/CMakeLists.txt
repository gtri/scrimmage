# scrimmage-viz only builds with VTK viewer (requires VTK network interface features)
if(BUILD_VTK_VIEWER)
  set (APP_NAME scrimmage-viz)

  SET(SRCS main.cpp )

  if(APPLE)
    add_executable(${APP_NAME} MACOSX_BUNDLE ${SRCS})
  else()
    add_executable(${APP_NAME} ${SRCS})
  endif()

  add_dependencies(${APP_NAME} scrimmage-protos)

  target_link_libraries(${APP_NAME}
    ${VTK_LIBRARIES}
    scrimmage-core
    scrimmage-viewer
    scrimmage-protos
  )

  if (NOT INSTALL_LINK)
      install(TARGETS ${APP_NAME} DESTINATION bin)
  endif()
endif()
