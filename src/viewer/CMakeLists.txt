###################################################
# Viewer Library
###################################################
set(LIBRARY_NAME scrimmage-viewer)
SET (LIB_MAJOR 0)
SET (LIB_MINOR 0)
SET (LIB_RELEASE 1)
SET (_soversion ${LIB_MAJOR}.${LIB_MINOR}.${LIB_RELEASE})

file (GLOB SRCS *.cpp)
ADD_LIBRARY(${LIBRARY_NAME} SHARED
  ${SRCS}
  )

add_dependencies(${LIBRARY_NAME} 
    scrimmage-core
    scrimmage-protos)

if(VTK_LIBRARIES)
  target_link_libraries(${LIBRARY_NAME}
    scrimmage-core
    scrimmage-protos
    scrimmage-boost
    ${VTK_LIBRARIES})
else()
  target_link_libraries(${LIBRARY_NAME}
    scrimmage-core
    scrimmage-protos
    scrimmage-boost
    vtkHybrid
    vtkWidgets)
endif()

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

# EDIT: scrimmage-viewer isn't part of core scrimmage anymore
# Push up the PROJECT_LIBS variable
#set(PROJECT_LIBS ${PROJECT_LIBS} ${LIBRARY_NAME} PARENT_SCOPE)
