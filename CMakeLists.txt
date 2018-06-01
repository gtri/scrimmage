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

# fetch all the local directories for generated code
get_filename_component(SCRIMMAGE_LIB_DIR ${CMAKE_BINARY_DIR}/lib ABSOLUTE)
get_filename_component(PROJECT_PLUGIN_LIBS_DIR ${CMAKE_BINARY_DIR}/plugin_libs ABSOLUTE)
get_filename_component(SCRIMMAGE_BIN_DIR ${CMAKE_BINARY_DIR}/bin ABSOLUTE)
get_filename_component(SCRIMMAGE_INC_DIR ./include ABSOLUTE)
get_filename_component(SCRIMMAGE_SRC_DIR ./src ABSOLUTE)
get_filename_component(SCRIMMAGE_SHARE_DIR ./share ABSOLUTE)
get_filename_component(SCRIMMAGE_SCRIPTS_DIR ./scripts ABSOLUTE)
get_filename_component(SCRIMMAGE_SETUP_DIR ./setup ABSOLUTE)
get_filename_component(PROJECT_MODULES_DIR ./cmake/Modules ABSOLUTE)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_MODULES_DIR})

set(SCRIMMAGE_PYTHON_DIR ${CMAKE_SOURCE_DIR}/python)

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

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall") # -fprofile-arcs -ftest-coverage")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wall") # -fprofile-arcs -ftest-coverage")

set(EXECUTABLE_OUTPUT_PATH ${SCRIMMAGE_BIN_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${SCRIMMAGE_LIB_DIR})

## set the cmake defaults for libraries and binaries
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${SCRIMMAGE_LIB_DIR} CACHE PATH
  "Output directory for the dynamic libraries" )
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${SCRIMMAGE_BIN_DIR} CACHE PATH
  "Output directory for the binaries" )
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${SCRIMMAGE_LIB_DIR} CACHE PATH
  "Output directory for the static libraries (archives)" )

option(SETUP_HOME_CONFIG "Setup ~/.scrimmage" ON)
if (SETUP_HOME_CONFIG)
  set(SCRIMMAGE_LOCAL_CONFIG_DIR "$ENV{HOME}/.scrimmage")
  set(SCRIMMAGE_ENV_DIR "${SCRIMMAGE_LOCAL_CONFIG_DIR}/env")
  file(MAKE_DIRECTORY ${SCRIMMAGE_ENV_DIR})
  configure_file(${CMAKE_SOURCE_DIR}/cmake/Modules/setenv.in
    "${SCRIMMAGE_ENV_DIR}/${PROJECT_NAME}-setenv" @ONLY)
  execute_process(COMMAND bash "-c" "${SCRIMMAGE_SETUP_DIR}/edit-config.sh ${SCRIMMAGE_LOCAL_CONFIG_DIR} ${PROJECT_NAME}" OUTPUT_VARIABLE result)
endif()

set(MSGS_INCLUDE_DIR ${CMAKE_BINARY_DIR}/msgs)
set(PROTO_INCLUDE_DIR ${CMAKE_BINARY_DIR}/src/proto)

include_directories(${SCRIMMAGE_INC_DIR})
include_directories(${MSGS_INCLUDE_DIR})
include_directories(${PROTO_INCLUDE_DIR})

option(EXTERNAL "EXTERNAL" 0)

option(ENABLE_PYTHON_BINDINGS "ENABLE_PYTHON_BINDINGS" ON)
set(PYTHON_MIN_VERSION "2.7" CACHE STRING "Minimum Python version for building")
if (ENABLE_PYTHON_BINDINGS)
  add_definitions(-DENABLE_PYTHON_BINDINGS=1)

  # Find Python
  #The order of the followin find_package calls is important!
  find_package(PythonInterp ${PYTHON_MIN_VERSION} REQUIRED)
  find_package(PythonLibs ${PYTHON_MIN_VERSION} REQUIRED)

  message(STATUS "Python Versions Found: ${PYTHONLIBS_VERSION_STRING}")
  message(STATUS "PYTHON_LIBRARIES : ${PYTHON_LIBRARIES}")
  message(STATUS "PYTHON_INCLUDE_DIRS: ${PYTHON_INCLUDE_DIRS}")

  set(PYTHON_FOUND TRUE)
  find_package(pybind11 REQUIRED)
  include_directories(${pybind11_INCLUDE_DIRS})
  set(PYBIND11_INCLUDE_DIRS ${pybind11_INCLUDE_DIRS})
  include_directories("${PYTHON_INCLUDE_DIRS}")

else()
  add_definitions(-DENABLE_PYTHON_BINDINGS=0)
endif()

########################################################
# Find Protobuf
########################################################
set(PROTOBUF_VERSION 3.3.0)
set(PROTOC "" CACHE STRING "External protoc executable")
set(PROTOBUF_LIBS "" CACHE STRING "External protobuf lib (libprotobuf.so)")
set(PROTOBUF_INCLUDE "" CACHE STRING "External protobuf include directory")
if (PROTOC STREQUAL "" OR PROTOBUF_LIBS STREQUAL "" OR PROTOBUF_INCLUDE STREQUAL "")
  find_package(Protobuf ${PROTOBUF_VERSION} REQUIRED)
else()
  set(PROTOBUF_PROTOC_EXECUTABLE "${PROTOC}")
  set(PROTOBUF_LIBRARIES "${PROTOBUF_LIBS}")
  set(PROTOBUF_INCLUDE_DIRS "${PROTOBUF_INCLUDE}")
endif()
include_directories(${PROTOBUF_INCLUDE_DIRS})
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
include_directories (${GeographicLib_INCLUDE_DIRS})

########################################################
# Find JSBSim
########################################################
find_package(JSBSIM REQUIRED)
string(FIND "${JSBSIM_INCLUDE_DIRS}" "NOTFOUND" POS)
if (JSBSIM_FOUND AND (POS EQUAL -1))
  include_directories(${JSBSIM_INCLUDE_DIR})
  add_definitions(-DENABLE_JSBSIM=1)
else()
  add_definitions(-DENABLE_JSBSIM=0)
  message(--------------------------)
  message("JSBSIM_NOT_FOUND, plugins that use this library will not be built")
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
include_directories(${EIGEN3_INCLUDE_DIR})

########################################################
# Find Boost
########################################################
#set(Boost_DEBUG ON)
find_package(Boost 1.55 COMPONENTS thread date_time iostreams program_options regex
  filesystem system graph REQUIRED)

########################################################
# Find OpenCV
########################################################
find_package(OpenCV QUIET)
if (OpenCV_FOUND)
  add_definitions(-DENABLE_OPENCV=1)
  include_directories(${OpenCV_INCLUDE_DIRS})
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
      configure_file(${CMAKE_SOURCE_DIR}/cmake/Modules/CMakeLists.txt.gtest.in
        googletest-download/CMakeLists.txt)
      execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
      if(result)
        message(FATAL_ERROR "CMake step for googletest failed: ${result}")
      endif()
      execute_process(COMMAND ${CMAKE_COMMAND} --build .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
      if(result)
        message(FATAL_ERROR "Build step for googletest failed: ${result}")
      endif()

      # Prevent overriding the parent project's compiler/linker
      # settings on Windows
      set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

      # Add googletest directly to our build. This defines
      # the gtest and gtest_main targets.
      add_subdirectory(${CMAKE_BINARY_DIR}/googletest-src
                       ${CMAKE_BINARY_DIR}/googletest-build)

      # The gtest/gtest_main targets carry header search path
      # dependencies automatically when using CMake 2.8.11 or
      # later. Otherwise we have to add them here ourselves.
      if (CMAKE_VERSION VERSION_LESS 2.8.11)
        include_directories("${gtest_SOURCE_DIR}/include")
      endif()
  endif()

  add_subdirectory(test)
endif()

###############################################################################
# Installation
###############################################################################
set(PROJECT_DESCRIPTION "A multi-agent simulator")

if (UNIX AND NOT APPLE)
  # - Add each library to LIBS_STRING PROJECT_LIBS is a "global" variable that
  # is passed down to each sub directory and every library that is generated in
  # this project appends its name to this variable.
  foreach(I ${PROJECT_LIBS})
    set (LIBS_STRING "${LIBS_STRING} -l${I}")
  endforeach()

  #############################################################################
  # Generate the cmake configuration files for the build tree
  #############################################################################
  include(CMakePackageConfigHelpers)

  # Configure variables used in project-config.cmake.in for build tree
  set(TMP_INCLUDE_INSTALL_DIR ${CMAKE_SOURCE_DIR}/include)
  set(TMP_MSGS_INCLUDE_DIR ${MSGS_INCLUDE_DIR})
  set(TMP_PROTO_INCLUDE_DIR ${PROTO_INCLUDE_DIR})

  configure_package_config_file(
    ${CMAKE_SOURCE_DIR}/cmake/Modules/project-config.cmake.in
    ${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config.cmake
    INSTALL_DESTINATION ${CMAKE_SOURCE_DIR}/build
    PATH_VARS TMP_INCLUDE_INSTALL_DIR
    )

  write_basic_package_version_file(
    ${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
    )

  # Add all library targets to the build-tree export set
  export(TARGETS ${PROJECT_LIBS} ${PROJECT_PLUGINS}
    FILE "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-targets.cmake")

  # Export the package for use from the build-tree
  # (this registers the build-tree with a global CMake-registry ~/.cmake)
  export(PACKAGE ${PROJECT_NAME})

  #############################################################################
  # Generate the cmake configuration files for the install tree
  #############################################################################
  set(INCLUDE_INSTALL_DIR include/ CACHE PATH "Include install dir")
  set(LIB_INSTALL_DIR lib/ CACHE PATH "Library install dir")
  set(CMAKE_CONFIG_DEST "${LIB_INSTALL_DIR}/${PROJECT_NAME}/cmake")

  # Configure variables used in project-config.cmake.in for install tree
  set(TMP_INCLUDE_INSTALL_DIR ${INCLUDE_INSTALL_DIR})
  unset(TMP_MSGS_INCLUDE_DIR)
  unset(TMP_PROTO_INCLUDE_DIR)

  configure_package_config_file(
    ${CMAKE_SOURCE_DIR}/cmake/Modules/project-config.cmake.in
    ${CMAKE_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}-config.cmake
    INSTALL_DESTINATION ${CMAKE_CONFIG_DEST}
    PATH_VARS TMP_INCLUDE_INSTALL_DIR
    )

  write_basic_package_version_file(
    ${CMAKE_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}-config-version.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
    )

  # Install cmake config files
  install(FILES ${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}-config.cmake
    ${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}-config-version.cmake
    DESTINATION ${CMAKE_CONFIG_DEST})

  install(
    EXPORT ${PROJECT_NAME}-targets
    DESTINATION ${CMAKE_CONFIG_DEST}
    )

  #############################################################################
  # Install Targets
  #############################################################################
  # Install all mission XML files under etc
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

  if (NOT EXTERNAL)
    # Install meshes, terrain, maps that come with scrimmage core
    install(
      DIRECTORY data/gui
      DESTINATION share/${PROJECT_NAME}/data
      )
  endif()

endif()

###############################################################################
# Binary installation generation
###############################################################################
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "SCRIMMAGE")
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME ${PROJECT_NAME})
set(CPACK_PACKAGE_RELEASE 1)
set(CPACK_PACKAGE_CONTACT "Kevin DeMarco")
set(CPACK_PACKAGE_VENDOR "GTRI")
set(CPACK_PACKAGING_INSTALL_PREFIX "/usr")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_PACKAGE_RELEASE}.${CMAKE_SYSTEM_PROCESSOR}")
include(CPack)
