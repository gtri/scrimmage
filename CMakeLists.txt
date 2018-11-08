cmake_minimum_required (VERSION 3.5)

option(ADD_HOME_LOCAL_PREFIX "Add ~/.local to cmake prefix" OFF)
if(ADD_HOME_LOCAL_PREFIX)
  set(CMAKE_PREFIX_PATH $ENV{HOME}/.local/ ${CMAKE_PREFIX_PATH})
endif()

# see : https://crascit.com/2016/04/09/using-ccache-with-cmake/
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
else()
  message("ccache not found, repeated builds will be faster if this program is installed")
endif()

project (scrimmage)

set(PROJECT_VERSION "0.2.0-dev")

# give SCRIMMAGE 1 series a few more soversions
set(SCRIMMAGE_SOVERSION "00")

if(NOT CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX /usr/local)
endif()

set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib:$ORIGIN/../lib/${PROJECT_NAME}/plugin_libs" CACHE STRING "The installed RPATH")

# fetch all the local directories for generated code
get_filename_component(PROJECT_PLUGIN_LIBS_DIR ${PROJECT_BINARY_DIR}/plugin_libs ABSOLUTE)
get_filename_component(SCRIMMAGE_INC_DIR ./include ABSOLUTE)
get_filename_component(SCRIMMAGE_SCRIPTS_DIR ./scripts ABSOLUTE)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/Modules
  ${PROJECT_SOURCE_DIR}/share/cmake/${PROJECT_NAME})

set(SCRIMMAGE_PYTHON_DIR ${PROJECT_SOURCE_DIR}/python)
set(SCRIMMAGE_PROTO_PATH ${CMAKE_CURRENT_LIST_DIR}/src/proto)
set(MSGS_INCLUDE_DIR ${PROJECT_BINARY_DIR}/msgs)
set(PROTO_INCLUDE_DIR ${PROJECT_BINARY_DIR}/src/proto)

include_directories(${PROTO_INCLUDE_DIR})

include(MacroAddExternalTarget)
include(MacroSubDirList)
include(FindPythonModule)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING
    "Choose the type of build, options are: Debug Release
      RelWithDebInfo MinSizeRel." FORCE)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# macOS requires additional directories added to the path
set(APPEND_TO_PATH "")
if(APPLE)
  set(CMAKE_SHARED_LIBRARY_CREATE_CXX_FLAGS "${CMAKE_SHARED_LIBRARY_CREATE_CXX_FLAGS} -undefined dynamic_lookup")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-return-type-c-linkage")
  string(JOIN ":" APPEND_TO_PATH
    "${PROJECT_BINARY_DIR}/bin/scrimmage.app/Contents/MacOS"
    "${PROJECT_BINARY_DIR}/bin/scrimmage-playback.app/Contents/MacOS"
    "${PROJECT_BINARY_DIR}/bin/scrimmage-viz.app/Contents/MacOS")
endif()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall") # -fprofile-arcs -ftest-coverage")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wall") # -fprofile-arcs -ftest-coverage")

option(SETUP_HOME_CONFIG "Setup ~/.scrimmage" ON)
set(LOCAL_CONFIG_DIR "$ENV{HOME}/.scrimmage" CACHE PATH "Local ~/.scrimmage config directory")
include(GenerateSetEnv)

GenerateSetEnv(
  SETUP_LOCAL_CONFIG_DIR ${SETUP_HOME_CONFIG}
  LOCAL_CONFIG_DIR ${LOCAL_CONFIG_DIR}
  SETENV_IN_FILE   ${PROJECT_SOURCE_DIR}/share/cmake/scrimmage/setenv.in
  MISSION_PATH     ${PROJECT_SOURCE_DIR}/missions
  PLUGIN_PATH      ${PROJECT_BINARY_DIR}/plugin_libs
                   ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/plugins
  PATH             ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} ${SCRIMMAGE_SCRIPTS_DIR} ${APPEND_TO_PATH}
  CONFIG_PATH      ${PROJECT_SOURCE_DIR}/config
  DATA_PATH        ${PROJECT_SOURCE_DIR}/data
  PYTHONPATH       ${PROJECT_SOURCE_DIR}/src/plugins/autonomy/python
                   ${PROJECT_BINARY_DIR}/msgs/scrimmage/msgs
                   )

option(EXTERNAL "EXTERNAL" 0)

option(ENABLE_PYTHON_BINDINGS "ENABLE_PYTHON_BINDINGS" ON)
set(PYTHON_MIN_VERSION "2.7" CACHE STRING "Minimum Python version for building")
if (ENABLE_PYTHON_BINDINGS)

  # Find Python
  #The order of the followin find_package calls is important!
  find_package(PythonInterp ${PYTHON_MIN_VERSION} REQUIRED)
  find_package(PythonLibs ${PYTHON_MIN_VERSION} REQUIRED)

  message(STATUS "Python Versions Found: ${PYTHONLIBS_VERSION_STRING}")
  message(STATUS "PYTHON_LIBRARIES : ${PYTHON_LIBRARIES}")
  message(STATUS "PYTHON_INCLUDE_DIRS: ${PYTHON_INCLUDE_DIRS}")

  set(PYTHON_FOUND TRUE)

  # We have already found the python libs and modules, so tell pybind11 to not
  # search for the python interpreter and libraries again.
  set(PYTHONLIBS_FOUND TRUE)
  set(PYTHON_MODULE_EXTENSION TRUE)
  find_package(pybind11 REQUIRED)
  set(PYBIND11_INCLUDE_DIRS ${pybind11_INCLUDE_DIRS})

  add_library(scrimmage-python INTERFACE)
  target_include_directories(scrimmage-python
      SYSTEM INTERFACE ${PYTHON_INCLUDE_DIRS} ${pybind11_INCLUDE_DIRS})
  target_compile_definitions(scrimmage-python
      INTERFACE -DENABLE_PYTHON_BINDINGS=1)
  target_link_libraries(scrimmage-python INTERFACE ${PYTHON_LIBRARIES})
  install(TARGETS scrimmage-python EXPORT ${PROJECT_NAME}-targets)
else()
  target_compile_definitions(scrimmage-python
      INTERFACE -DENABLE_PYTHON_BINDINGS=0)
endif()

########################################################
# Find Protobuf
########################################################
set(PROTOBUF_VERSION 3.0)
set(PROTOC "" CACHE STRING "External protoc executable")
set(PROTOBUF_LIBS "" CACHE STRING "External protobuf lib (libprotobuf.so)")
set(PROTOBUF_INCLUDE "" CACHE STRING "External protobuf include directory")
if (PROTOC STREQUAL "" OR PROTOBUF_LIBS STREQUAL "" OR PROTOBUF_INCLUDE STREQUAL "")
  find_package(Protobuf ${PROTOBUF_VERSION} REQUIRED)
  if (Protobuf_VERSION AND CMAKE_VERSION VERSION_GREATER 3.5)
      set(PROTOBUF_VERSION ${Protobuf_VERSION})
  else()
      # cmake does not provide a version before 3.6
      # so we assume that it is the protobuf supplied by the scrimmage ppa
      # this is only used for versioning the protobuf in the setup.py file
      # so it can be overwritten
      set(PROTOBUF_VERSION 3.3.0)
  endif()
else()
  set(PROTOBUF_PROTOC_EXECUTABLE "${PROTOC}")
  set(PROTOBUF_LIBRARIES "${PROTOBUF_LIBS}")
  set(PROTOBUF_INCLUDE_DIRS "${PROTOBUF_INCLUDE}")
endif()
set(PROTOBUF_INCLUDE ${PROTOBUF_INCLUDE_DIRS})

########################################################
# Find GRPC
########################################################
set(GRPC_VERSION 1.2.1)
find_package(GRPC)
if (GRPC_FOUND)
  add_definitions(-DENABLE_GRPC=1)
else()
  add_definitions(-DENABLE_GRPC=0)
endif()

########################################################
# Find GeographicLib
########################################################
find_package (GeographicLib REQUIRED)
add_definitions (${GeographicLib_DEFINITIONS})

########################################################
# Find JSBSim
########################################################
if (NOT APPLE)
  find_package(JSBSIM REQUIRED)
  string(FIND "${JSBSIM_INCLUDE_DIRS}" "NOTFOUND" POS)
  if (JSBSIM_FOUND AND (POS EQUAL -1))
    add_definitions(-DENABLE_JSBSIM=1)
  else()
    add_definitions(-DENABLE_JSBSIM=0)
    message(--------------------------)
    message("JSBSIM_NOT_FOUND, plugins that use this library will not be built")
    message(--------------------------)
  endif()
else()
  add_definitions(-DENABLE_JSBSIM=0)
  message(--------------------------)
  message("Apple detected, plugins that use JSBSim will not be built")
  message(--------------------------)
endif()

########################################################
# Find VTK
########################################################
find_package(VTK QUIET NO_MODULE COMPONENTS
    vtkFiltersSources
    vtkFiltersTexture
    vtkInteractionStyle
    vtkIOGeometry
    vtkIOImage
    vtkIOLegacy
    vtkRenderingAnnotation
    vtkRenderingCore
    vtkRenderingOpenGL
    )

if (NOT EXTERNAL AND ${VTK_FOUND})
  message(STATUS "Found VTK Version: ${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.${VTK_BUILD_VERSION}")
  include_directories(SYSTEM ${VTK_INCLUDE_DIRS})
  include(${VTK_USE_FILE})
  add_definitions(-DENABLE_VIEWER=1)
else()
  add_definitions(-DENABLE_VIEWER=0)
endif()

########################################################
# Find Eigen
########################################################
if(NOT "${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
  # 32 bit check, see https://cmake.org/pipermail/cmake/2013-December/056587.html
  # see also: http://eigen.tuxfamily.org/dox/group__TopicUnalignedArrayAssert.html
  add_definitions(-DEIGEN_DONT_VECTORIZE=1)
  add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT=1)
endif()

find_package(Eigen3 QUIET)

if (NOT Eigen3_FOUND)
  include(LibFindMacros)
  libfind_pkg_check_modules(EIGEN_PKGCONFIG eigen3)
  if (EIGEN_PKGCONFIG_INCLUDE_DIRS STREQUAL "")
    message(FATAL_ERROR "could not find eigen either through find_package or pkgconfig")
  endif()
  set(EIGEN3_INCLUDE_DIR ${EIGEN_PKGCONFIG_INCLUDE_DIRS})
  message("Found Eigen with pkg-config, EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")
endif()

########################################################
# Find Boost
########################################################
#set(Boost_DEBUG ON)
find_package(Boost 1.55 COMPONENTS thread date_time iostreams program_options regex
  filesystem system graph REQUIRED)
add_library(scrimmage-boost INTERFACE)
target_include_directories(scrimmage-boost INTERFACE ${Boost_INCLUDE_DIRS})
target_link_libraries(scrimmage-boost INTERFACE ${Boost_LIBRARIES})
install(TARGETS scrimmage-boost EXPORT ${PROJECT_NAME}-targets)

########################################################
# Find OpenCV
########################################################
find_package(OpenCV QUIET)
if (OpenCV_FOUND)
  add_library(scrimmage-opencv INTERFACE)
  target_include_directories(scrimmage-opencv INTERFACE ${OpenCV_INCLUDE_DIRS})
  target_compile_definitions(scrimmage-opencv INTERFACE -DENABLE_OPENCV=1)
  install(TARGETS scrimmage-opencv EXPORT ${PROJECT_NAME}-targets)
else()
  add_definitions(-DENABLE_OPENCV=0)
endif()

########################################################
# Find ROS
########################################################
option(BUILD_ROS_PLUGINS "BUILD_ROS_PLUGINS" OFF)
if (BUILD_ROS_PLUGINS)
  set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /opt/ros/kinetic)
  find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs
    geometry_msgs nav_msgs tf)
  if (catkin_FOUND)
    include_directories(${catkin_INCLUDE_DIRS})
  endif()
endif()

########################################################
# Find AirSim
########################################################
find_package(AirSim)
if (AIRSIM_FOUND)
  include_directories(${AIRSIM_INCLUDE_DIRS})
  add_definitions(-DENABLE_AIRSIM=1)
else()
  add_definitions(-DENABLE_AIRSIM=0)
endif()


########################################################
# Recurse into src, share, and plugins
########################################################
add_subdirectory(share)
add_subdirectory(msgs)
add_subdirectory(src)
if (NOT EXTERNAL)
  add_subdirectory(tools)
endif()
add_subdirectory(${SCRIMMAGE_PYTHON_DIR})

option(BUILD_DOCS "Build documentation" OFF)
if (BUILD_DOCS)
  add_subdirectory(docs)
endif()

###################################################################
# Add gtest
###################################################################
option(BUILD_TESTS "BUILD_TESTS" OFF)
if (NOT DEFINED CMAKE_TOOLCHAIN_FILE AND BUILD_TESTS)
  enable_testing()

  ############################################################
  # copied from
  # https://github.com/google/googletest/tree/master/googletest#incorporating-into-an-existing-cmake-project
  ############################################################

  # Download and unpack googletest at configure time
  if (NOT TARGET gtest_main)
      configure_file(${PROJECT_SOURCE_DIR}/cmake/Modules/CMakeLists.txt.gtest.in
        googletest-download/CMakeLists.txt)
      execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/googletest-download )
      if(result)
        message(FATAL_ERROR "CMake step for googletest failed: ${result}")
      endif()
      execute_process(COMMAND ${CMAKE_COMMAND} --build .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/googletest-download )
      if(result)
        message(FATAL_ERROR "Build step for googletest failed: ${result}")
      endif()

      # Prevent overriding the parent project's compiler/linker
      # settings on Windows
      set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

      # Add googletest directly to our build. This defines
      # the gtest and gtest_main targets.
      add_subdirectory(${PROJECT_BINARY_DIR}/googletest-src
                       ${PROJECT_BINARY_DIR}/googletest-build)
  endif()

  add_subdirectory(test)
endif()

###############################################################################
# Installation
###############################################################################
set(PROJECT_DESCRIPTION "A multi-agent simulator")

# - Add each library to LIBS_STRING
# SCRIMMAGE_GTRI_SIM_LIBS is a "global" variable that is passed
# down to each sub directory and every library that is
# generated in this project appends its name to this
# variable.
foreach(I ${PROJECT_LIBS})
  set (LIBS_STRING "${LIBS_STRING} -l${I}")
endforeach()

#############################################################################
# Generate the cmake configuration files for the build tree
#############################################################################
include(CMakePackageConfigHelpers)

# Configure variables used in project-config.cmake.in for build tree
set(TMP_INCLUDE_INSTALL_DIR ${PROJECT_SOURCE_DIR}/include)
set(TMP_MSGS_INCLUDE_DIR ${MSGS_INCLUDE_DIR})
set(TMP_PROTO_INCLUDE_DIR ${PROTO_INCLUDE_DIR})
set(SCRIMMAGE_MSGS_PROTO_PATH ${CMAKE_CURRENT_LIST_DIR}/msgs)
set(SHARE_INSTALL_DIR ${PROJECT_SOURCE_DIR}/share)

configure_package_config_file(
  ${PROJECT_SOURCE_DIR}/cmake/Modules/project-config.cmake.in
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION ${PROJECT_BINARY_DIR}
  PATH_VARS TMP_INCLUDE_INSTALL_DIR SCRIMMAGE_PROTO_PATH
  SCRIMMAGE_MSGS_PROTO_PATH SHARE_INSTALL_DIR
  )

write_basic_package_version_file(
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
  )

# Export project export set to the build tree
export(
  EXPORT ${PROJECT_NAME}-targets
  FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake"
  )

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry ~/.cmake)
export(PACKAGE ${PROJECT_NAME})

#############################################################################
# Generate the cmake configuration files for the install tree
#############################################################################
set(INCLUDE_INSTALL_DIR include/ CACHE PATH "Include install dir")
set(SHARE_INSTALL_DIR share/)
set(LIB_INSTALL_DIR lib/ CACHE PATH "Library install dir")
set(CMAKE_CONFIG_DEST "${LIB_INSTALL_DIR}/${PROJECT_NAME}/cmake")

# Configure variables used in project-config.cmake.in for install tree
set(TMP_INCLUDE_INSTALL_DIR ${INCLUDE_INSTALL_DIR})
unset(TMP_MSGS_INCLUDE_DIR)
unset(TMP_PROTO_INCLUDE_DIR)
set(SCRIMMAGE_PROTO_PATH ${SHARE_INSTALL_DIR}/${PROJECT_NAME}/proto)
set(SCRIMMAGE_MSGS_PROTO_PATH ${SHARE_INSTALL_DIR}/${PROJECT_NAME}/msgs)

configure_package_config_file(
  ${PROJECT_SOURCE_DIR}/cmake/Modules/project-config.cmake.in
  ${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION ${CMAKE_CONFIG_DEST}
  PATH_VARS TMP_INCLUDE_INSTALL_DIR SCRIMMAGE_PROTO_PATH
  SCRIMMAGE_MSGS_PROTO_PATH SHARE_INSTALL_DIR
  )

write_basic_package_version_file(
  ${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}-config-version.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
  )

# Install cmake config files
install(FILES ${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}-config.cmake
  ${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}-config-version.cmake
  DESTINATION ${CMAKE_CONFIG_DEST})

# Export project export set to the install tree
install(
  EXPORT ${PROJECT_NAME}-targets
  DESTINATION ${CMAKE_CONFIG_DEST}
  )

#############################################################################
# Install Targets
#############################################################################
# Install all mission XML files under share
install(
  DIRECTORY missions
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.xml"
  )

# Install all library headers
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  )

# Install all plugin configuration files
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION etc
  FILES_MATCHING PATTERN "*.xml"
  )

# Install all scrimmage proto headers
install(
  DIRECTORY ${PROTO_INCLUDE_DIR}/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.pb.h"
  )

# Install all project message proto headers
install(
  DIRECTORY ${MSGS_INCLUDE_DIR}/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.pb.h"
  )

# Install the .proto files in case another project wants to use them
install(
  DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/src/proto
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.proto"
  )

# Install the message .proto files in case another project wants to use them
install(
  DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/msgs
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.proto"
  )

if (NOT EXTERNAL)
  # Install meshes, terrain, maps that come with scrimmage core
  install(
    DIRECTORY data/gui
    DESTINATION share/${PROJECT_NAME}/data
    )
endif()

# Create the ld.so.conf.d/project.conf file
if (NOT APPLE)
  file(WRITE
    ${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}.conf
    "${CMAKE_INSTALL_PREFIX}/lib\n${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/plugin_libs"
    )
  option(ENABLE_LD_SO_CONF_INSTALL "Install ${PROJECT_NAME}.conf to /etc/ld.so.conf.d" OFF)
  if (ENABLE_LD_SO_CONF_INSTALL)
    install(
      FILES ${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}.conf
      DESTINATION /etc/ld.so.conf.d
      )
  endif()
endif()

# Install the project-setenv file
install(
  FILES ${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}-setenv
  DESTINATION etc/scrimmage/env
  )

# Install scrimmage cmake helper files
install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/share/cmake
  DESTINATION share
  )

###############################################################################
# Binary installation generation
###############################################################################
set(CPACK_GENERATOR "DEB")
set(CPACK_INSTALL_CMAKE_PROJECTS "${PROJECT_BINARY_DIR};${PROJECT_NAME};ALL;/")
set(CPACK_OUTPUT_CONFIG_FILE "${PROJECT_BINARY_DIR}/CPackConfig.cmake")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "SCRIMMAGE")
set(CPACK_PACKAGE_NAME ${PROJECT_NAME})
set(CPACK_PACKAGE_RELEASE 1)
set(CPACK_PACKAGE_CONTACT "Kevin DeMarco")
set(CPACK_PACKAGE_VENDOR "GTRI")
set(CPACK_PACKAGING_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_PACKAGE_RELEASE}.${CMAKE_SYSTEM_PROCESSOR}")
include(CPack)
