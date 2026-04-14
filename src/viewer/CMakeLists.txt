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
  set(OGRE_NEXT_GL3PLUS_PLUGIN_HINTS
    ${OGRE_LIBRARY_DIRS}
    /usr/lib/OGRE-Next
    /usr/local/lib/OGRE-Next)

  if (CMAKE_LIBRARY_ARCHITECTURE)
    list(APPEND OGRE_NEXT_GL3PLUS_PLUGIN_HINTS
      /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/OGRE-Next
      /usr/local/lib/${CMAKE_LIBRARY_ARCHITECTURE}/OGRE-Next)
  endif()

  set(OGRE_NEXT_LIBRARY_HINTS
    ${OGRE_LIBRARY_DIRS}
    /usr/lib
    /usr/local/lib)

  if (CMAKE_LIBRARY_ARCHITECTURE)
    list(APPEND OGRE_NEXT_LIBRARY_HINTS
      /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}
      /usr/local/lib/${CMAKE_LIBRARY_ARCHITECTURE})
  endif()

  find_file(OGRE_NEXT_GL3PLUS_PLUGIN_PATH
    NAMES RenderSystem_GL3Plus.so
    HINTS ${OGRE_NEXT_GL3PLUS_PLUGIN_HINTS})

  find_library(OGRE_NEXT_HLMS_PBS_LIBRARY
    NAMES OgreNextHlmsPbs
    HINTS ${OGRE_NEXT_LIBRARY_HINTS})

  find_library(OGRE_NEXT_HLMS_UNLIT_LIBRARY
    NAMES OgreNextHlmsUnlit
    HINTS ${OGRE_NEXT_LIBRARY_HINTS})

  if (NOT OGRE_NEXT_GL3PLUS_PLUGIN_PATH)
    message(FATAL_ERROR "SCRIMMAGE_HAVE_OGRE_NEXT_VIEWER is ON but RenderSystem_GL3Plus was not found")
  endif()

  if (NOT OGRE_NEXT_HLMS_PBS_LIBRARY OR NOT OGRE_NEXT_HLMS_UNLIT_LIBRARY)
    message(FATAL_ERROR "SCRIMMAGE_HAVE_OGRE_NEXT_VIEWER is ON but the Ogre-Next HLMS libraries were not found")
  endif()

  add_library(scrimmage-viewer-ogre-next SHARED
    ogre_next/OgreNextBootstrap.cpp
    ogre_next/OgreNextViewerBackend.cpp)

  add_dependencies(scrimmage-viewer-ogre-next
    scrimmage-core
    scrimmage-protos)

  target_include_directories(scrimmage-viewer-ogre-next
    PUBLIC ${VIEWER_PUBLIC_INCLUDE_DIRS}
    PRIVATE
      ${OGRE_INCLUDE_DIRS}/Hlms/Common
      ${OGRE_INCLUDE_DIRS}/Hlms/Pbs
      ${OGRE_INCLUDE_DIRS}/Hlms/Unlit)

  target_compile_definitions(scrimmage-viewer-ogre-next
    PRIVATE
      OGRE_DEBUG_MODE=0
      SCRIMMAGE_OGRE_NEXT_PLUGIN_PATH="${OGRE_NEXT_GL3PLUS_PLUGIN_PATH}"
      SCRIMMAGE_OGRE_NEXT_HLMS_SOURCE_PATH="${PROJECT_SOURCE_DIR}/data/ogre-next/Hlms"
      SCRIMMAGE_OGRE_NEXT_HLMS_INSTALL_PATH="${CMAKE_INSTALL_PREFIX}/share/scrimmage/ogre-next/Hlms")

  if (OGRE_LIBRARY_DIRS)
    target_link_directories(scrimmage-viewer-ogre-next PRIVATE ${OGRE_LIBRARY_DIRS})
  endif()

  target_link_libraries(scrimmage-viewer-ogre-next
    scrimmage-core
    scrimmage-protos
    scrimmage-boost
    ${OGRE_LIBRARIES}
    ${OGRE_NEXT_HLMS_PBS_LIBRARY}
    ${OGRE_NEXT_HLMS_UNLIT_LIBRARY})

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
