###################################################
# Viewer Library
###################################################
set(LIBRARY_NAME scrimmage-viewer)
SET (LIB_MAJOR 0)
SET (LIB_MINOR 0)
SET (LIB_RELEASE 1)
SET (_soversion ${LIB_MAJOR}.${LIB_MINOR}.${LIB_RELEASE})

# Build either Ogre or VTK viewer based on configuration
if(BUILD_OGRE_VIEWER)
  # Ogre3D-based viewer
  SET(SRCS
    ogre/CameraController.cpp
    ogre/ContactRenderer.cpp
    ogre/MaterialPool.cpp
    ogre/OgreViewer.cpp
    ogre/ShapeRenderer.cpp
  )

  ADD_LIBRARY(${LIBRARY_NAME} SHARED ${SRCS})

  add_dependencies(${LIBRARY_NAME}
    scrimmage-core
    scrimmage-protos)

  target_link_libraries(${LIBRARY_NAME}
    scrimmage-core
    scrimmage-protos
    scrimmage-boost
    ${OGRE_LIBRARIES}
  )

  target_include_directories(${LIBRARY_NAME} SYSTEM PUBLIC ${OGRE_INCLUDE_DIRS})

elseif(BUILD_VTK_VIEWER)
  # VTK-based viewer (original)
  SET(SRCS CameraInterface.cpp Grid.cpp OriginAxes.cpp Updater.cpp Viewer.cpp)

  ADD_LIBRARY(${LIBRARY_NAME} SHARED ${SRCS})

  add_dependencies(${LIBRARY_NAME}
    scrimmage-core
    scrimmage-protos)

  if(VTK_LIBRARIES)
    target_link_libraries(${LIBRARY_NAME}
      scrimmage-core
      scrimmage-protos
      scrimmage-boost
      ${VTK_LIBRARIES})
    if(${VTK_MAJOR_VERSION} EQUAL 9)
      vtk_module_autoinit(
        TARGETS ${LIBRARY_NAME}
        MODULES ${VTK_LIBRARIES})
    endif()
  else()
    target_link_libraries(${LIBRARY_NAME}
      scrimmage-core
      scrimmage-protos
      scrimmage-boost
      vtkHybrid
      vtkWidgets)
  endif()
else()
  # No viewer - create empty library for compatibility
  SET(SRCS)
  ADD_LIBRARY(${LIBRARY_NAME} SHARED ${SRCS})
  message(WARNING "No viewer backend available - scrimmage-viewer will be empty")
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
