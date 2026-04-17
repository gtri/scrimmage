###################################################
# Backend-neutral viewer entry layer
###################################################
set(VIEWER_FACADE_LIB scrimmage-viewer)
set(LIB_MAJOR 0)
set(LIB_MINOR 0)
set(LIB_RELEASE 1)
set(_soversion ${LIB_MAJOR}.${LIB_MINOR}.${LIB_RELEASE})

set(VIEWER_PUBLIC_INCLUDE_DIRS
  "$<INSTALL_INTERFACE:include>"
  "$<BUILD_INTERFACE:${SCRIMMAGE_INC_DIR}>")

add_library(${VIEWER_FACADE_LIB} SHARED
  VisualizationSession.cpp
  ViewerFactory.cpp)

target_include_directories(${VIEWER_FACADE_LIB}
  PUBLIC ${VIEWER_PUBLIC_INCLUDE_DIRS})

if (SCRIMMAGE_HAVE_VTK_VIEWER)
  target_compile_definitions(${VIEWER_FACADE_LIB} PRIVATE SCRIMMAGE_HAVE_VTK_VIEWER=1)
else()
  target_compile_definitions(${VIEWER_FACADE_LIB} PRIVATE SCRIMMAGE_HAVE_VTK_VIEWER=0)
endif()

if (SCRIMMAGE_HAVE_OGRE_NEXT_VIEWER)
  target_compile_definitions(${VIEWER_FACADE_LIB} PRIVATE SCRIMMAGE_HAVE_OGRE_NEXT_VIEWER=1)
else()
  target_compile_definitions(${VIEWER_FACADE_LIB} PRIVATE SCRIMMAGE_HAVE_OGRE_NEXT_VIEWER=0)
endif()

set_target_properties(
  ${VIEWER_FACADE_LIB}
  PROPERTIES
  SOVERSION ${LIB_MAJOR}
  VERSION ${_soversion})

install(TARGETS ${VIEWER_FACADE_LIB}
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib)

###################################################
# VTK backend
###################################################
if (SCRIMMAGE_HAVE_VTK_VIEWER)
  add_library(scrimmage-viewer-vtk SHARED
    vtk/VtkCameraInterface.cpp
    vtk/VtkGrid.cpp
    vtk/VtkOriginAxes.cpp
    vtk/VtkUpdater.cpp
    vtk/VtkViewer.cpp
    vtk/VtkViewerBackend.cpp)

  add_dependencies(scrimmage-viewer-vtk
    scrimmage-core
    scrimmage-protos)

  target_include_directories(scrimmage-viewer-vtk
    PUBLIC ${VIEWER_PUBLIC_INCLUDE_DIRS})

  if (VTK_LIBRARIES)
    target_link_libraries(scrimmage-viewer-vtk
      scrimmage-core
      scrimmage-protos
      scrimmage-boost
      ${VTK_LIBRARIES})
    if (${VTK_MAJOR_VERSION} EQUAL 9)
      vtk_module_autoinit(
        TARGETS scrimmage-viewer-vtk
        MODULES ${VTK_LIBRARIES})
    endif()
  else()
    target_link_libraries(scrimmage-viewer-vtk
      scrimmage-core
      scrimmage-protos
      scrimmage-boost
      vtkHybrid
      vtkWidgets)
  endif()

  target_link_libraries(${VIEWER_FACADE_LIB} PRIVATE scrimmage-viewer-vtk)

  set_target_properties(
    scrimmage-viewer-vtk
    PROPERTIES
    SOVERSION ${LIB_MAJOR}
    VERSION ${_soversion})

  install(TARGETS scrimmage-viewer-vtk
    EXPORT ${PROJECT_NAME}-targets
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib)
endif()

###################################################
# Ogre-Next backend
###################################################
if (SCRIMMAGE_HAVE_OGRE_NEXT_VIEWER)
  add_library(scrimmage-viewer-ogre-next SHARED
    ogre_next/OgreNextViewerBackend.cpp
    ogre_next/OgreNextViewer.cpp)

  add_dependencies(scrimmage-viewer-ogre-next
    scrimmage-core
    scrimmage-protos)

  target_include_directories(scrimmage-viewer-ogre-next
    PUBLIC ${VIEWER_PUBLIC_INCLUDE_DIRS}
    PRIVATE ${OGRE_INCLUDE_DIRS})

  if (OGRE_LIBRARY_DIRS)
    target_link_directories(scrimmage-viewer-ogre-next PRIVATE ${OGRE_LIBRARY_DIRS})
  endif()

  # Link OGRE libraries - pkg-config may not include HLMS libs, so add explicitly
  target_link_libraries(scrimmage-viewer-ogre-next
    scrimmage-core
    scrimmage-protos
    scrimmage-boost
    ${OGRE_LIBRARIES}
    OgreNextHlmsPbs
    OgreNextHlmsUnlit)

  target_link_libraries(${VIEWER_FACADE_LIB} PRIVATE scrimmage-viewer-ogre-next)

  set_target_properties(
    scrimmage-viewer-ogre-next
    PROPERTIES
    SOVERSION ${LIB_MAJOR}
    VERSION ${_soversion})

  install(TARGETS scrimmage-viewer-ogre-next
    EXPORT ${PROJECT_NAME}-targets
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib)
endif()
