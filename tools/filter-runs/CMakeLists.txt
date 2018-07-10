set (APP_NAME filter-runs)

file (GLOB SRCS *.cpp)
file (GLOB HDRS *.h)

add_executable(${APP_NAME} ${SRCS})

add_dependencies(${APP_NAME} scrimmage-protos)

target_link_libraries(${APP_NAME}
  scrimmage-core
  scrimmage-boost
  )
if (ENABLE_PYTHON_BINDINGS)
  target_link_libraries(${APP_NAME} scrimmage-python)
endif()

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
