set (APP_NAME scrimmage-viz)

file (GLOB SRCS *.cpp)
file (GLOB HDRS *.h)

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
#if (${VTK_FOUND})
#  target_link_libraries(${APP_NAME}
#    scrimmage-viewer
#    )
#endif()
