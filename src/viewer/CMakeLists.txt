###################################################
# Viewer Library
###################################################
set(LIBRARY_NAME scrimmage-viewer)
SET (LIB_MAJOR 0)
SET (LIB_MINOR 0)
SET (LIB_RELEASE 1)
SET (_soversion ${LIB_MAJOR}.${LIB_MINOR}.${LIB_RELEASE})

# Collect sources from all available viewer backends
SET(SRCS)
SET(VIEWER_LIBS)

# Add Ogre3D viewer sources if available
if(BUILD_OGRE_VIEWER)
  list(APPEND SRCS
    ogre/CameraController.cpp
    ogre/ContactRenderer.cpp
    ogre/MaterialPool.cpp
    ogre/OgreViewer.cpp
    ogre/ShapeRenderer.cpp
  )
  list(APPEND VIEWER_LIBS ${OGRE_LIBRARIES})
endif()

# Add VTK viewer sources if available
if(BUILD_VTK_VIEWER)
  list(APPEND SRCS
    CameraInterface.cpp
    Grid.cpp
    OriginAxes.cpp
    Updater.cpp
    Viewer.cpp
  )
  if(VTK_LIBRARIES)
    list(APPEND VIEWER_LIBS ${VTK_LIBRARIES})
  else()
    list(APPEND VIEWER_LIBS vtkHybrid vtkWidgets)
  endif()
endif()

# Create the library
if(SRCS)
  ADD_LIBRARY(${LIBRARY_NAME} SHARED ${SRCS})

  add_dependencies(${LIBRARY_NAME}
    scrimmage-core
    scrimmage-protos)

  target_link_libraries(${LIBRARY_NAME}
    scrimmage-core
    scrimmage-protos
    scrimmage-boost
    ${VIEWER_LIBS}
  )

  if(BUILD_OGRE_VIEWER)
    target_include_directories(${LIBRARY_NAME} SYSTEM PUBLIC ${OGRE_INCLUDE_DIRS})
  endif()

  if(BUILD_VTK_VIEWER AND VTK_LIBRARIES AND ${VTK_MAJOR_VERSION} EQUAL 9)
    vtk_module_autoinit(
      TARGETS ${LIBRARY_NAME}
      MODULES ${VTK_LIBRARIES})
  endif()
else()
  # No viewer - create empty library for compatibility
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
