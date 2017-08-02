cmake_minimum_required (VERSION 3.5)

option(ADD_HOME_LOCAL_PREFIX "Add ~/.local to cmake prefix" ON)
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

project ((>>>PROJECT_NAME<<<))

set(PROJECT_VERSION_MAJOR "0")
set(PROJECT_VERSION_MINOR "0")
set(PROJECT_VERSION_PATCH "0")
set(PROJECT_VERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}")
set(PROJECT_VERSION_DATE "2017.07.21")

# give PROJECT 1 series a few more soversions
set(PROJECT_SOVERSION "00")

if(NOT CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX /usr/local)
endif()

# fetch all the local directories for generated code
get_filename_component(PROJECT_LIB_DIR ${CMAKE_BINARY_DIR}/lib ABSOLUTE)
get_filename_component(PROJECT_PLUGIN_LIBS_DIR ${CMAKE_BINARY_DIR}/plugin_libs ABSOLUTE)
get_filename_component(PROJECT_BIN_DIR ${CMAKE_BINARY_DIR}/bin ABSOLUTE)
get_filename_component(PROJECT_INC_DIR ./include ABSOLUTE)
get_filename_component(PROJECT_SRC_DIR ./src ABSOLUTE)
get_filename_component(PROJECT_SHARE_DIR ./share ABSOLUTE)
get_filename_component(PROJECT_SCRIPTS_DIR ./scripts ABSOLUTE)
get_filename_component(PROJECT_SETUP_DIR ./setup ABSOLUTE)
get_filename_component(PROJECT_MODULES_DIR ./cmake/Modules ABSOLUTE)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_MODULES_DIR})

include(MacroSubDirList)

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

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BIN_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_LIB_DIR})

## set the cmake defaults for libraries and binaries
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_LIB_DIR} CACHE PATH
  "Output directory for the dynamic libraries" )
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BIN_DIR} CACHE PATH
  "Output directory for the binaries" )
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_LIB_DIR} CACHE PATH
  "Output directory for the static libraries (archives)" )

set(SCRIMMAGE_LOCAL_CONFIG_DIR "$ENV{HOME}/.scrimmage")
set(SCRIMMAGE_ENV_DIR "${SCRIMMAGE_LOCAL_CONFIG_DIR}/env")
file(MAKE_DIRECTORY ${SCRIMMAGE_ENV_DIR})
configure_file(${CMAKE_SOURCE_DIR}/cmake/Modules/setenv.in
  "${SCRIMMAGE_ENV_DIR}/${PROJECT_NAME}-setenv" @ONLY)
execute_process(COMMAND bash "-c" "${PROJECT_SETUP_DIR}/edit-config.sh ${SCRIMMAGE_LOCAL_CONFIG_DIR} ${PROJECT_NAME}" OUTPUT_VARIABLE result)

set(MSGS_INCLUDE_DIR ${CMAKE_BINARY_DIR}/msgs)
set(PROTO_INCLUDE_DIR ${CMAKE_BINARY_DIR}/src/proto)

include_directories(${PROJECT_INC_DIR})
include_directories(${MSGS_INCLUDE_DIR})
include_directories(${PROTO_INCLUDE_DIR})

###############################################################################
# Find Scrimmage
###############################################################################
find_package(scrimmage REQUIRED)
include_directories(${SCRIMMAGE_INCLUDE_DIRS})

###############################################################################
# Find geographiclib (required)
###############################################################################
find_package (GeographicLib 1.34 REQUIRED)
add_definitions (${GeographicLib_DEFINITIONS})
include_directories(${GeographicLib_INCLUDE_DIRS})

###############################################################################
# Find Protobuf (required)
###############################################################################
set(PROTOC "" CACHE STRING "External protoc executable")
set(PROTOBUF_LIBS "" CACHE STRING "External protobuf lib (libprotobuf.so)")
set(PROTOBUF_INCLUDE "" CACHE STRING "External protobuf include directory")
if (PROTOC STREQUAL "" OR PROTOBUF_LIBS STREQUAL "" OR PROTOBUF_INCLUDE STREQUAL "")
  find_package(Protobuf 3.3.0 REQUIRED)
else()
  set(PROTOBUF_PROTOC_EXECUTABLE "${PROTOC}")
  set(PROTOBUF_LIBRARIES "${PROTOBUF_LIBS}")
  set(PROTOBUF_INCLUDE_DIRS "${PROTOBUF_INCLUDE}")
endif()
include_directories(${PROTOBUF_INCLUDE_DIRS})

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
find_package(Boost COMPONENTS thread date_time iostreams program_options regex
  filesystem system REQUIRED)

include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

########################################################
# Handle Python settings
########################################################
if (SCRIMMAGE_ENABLE_PYTHON_BINDINGS)
  add_definitions(-DENABLE_PYTHON_BINDINGS=1)
  find_package(PythonLibs ${PYTHON_VERSION} REQUIRED)
  find_package(PythonInterp ${PYTHON_VERSION} REQUIRED)

  message(STATUS "Python Versions Found: ${PYTHONLIBS_VERSION_STRING}")
  message(STATUS "PYTHON_LIBRARIES : ${PYTHON_LIBRARIES}")
  message(STATUS "PYTHON_INCLUDE_DIRS: ${PYTHON_INCLUDE_DIRS}")

  set(PYTHON_FOUND TRUE)
  find_package(pybind11 REQUIRED)
  include_directories(${pybind11_INCLUDE_DIRS})
  include_directories("${PYTHON_INCLUDE_DIRS}")

else() 
  add_definitions(-DENABLE_PYTHON_BINDINGS=0)
endif() 

########################################################
# Find OpenCV
########################################################
find_package(OpenCV QUIET)
if (OPENCV_FOUND)  
  add_definitions(-DENABLE_OPENCV=1)
  include_directories(${OpenCV_INCLUDE_DIRS})
else()
  add_definitions(-DENABLE_OPENCV=0)  
endif()

########################################################
# Recurse into src, share, and plugins
########################################################
#add_subdirectory(share)
#add_subdirectory(msgs)
add_subdirectory(src)

option(BUILD_DOCS "Build documentation" OFF)
if (BUILD_DOCS)
  add_subdirectory(docs)
endif()

###################################################################
# Add gtest
###################################################################
option(BUILD_TESTS "BUILD_TESTS" OFF)
option(BUILD_GTEST "Build_GTEST" ON)
if (NOT DEFINED CMAKE_TOOLCHAIN_FILE AND BUILD_TESTS)
  enable_testing()
  
  if (BUILD_GTEST)
    ########################################################
    # Download and build googletest
    ########################################################
    include(ExternalProject)
    set(EXTERNAL_PROJECT_GTEST googletest)
    ExternalProject_Add (
      ${EXTERNAL_PROJECT_GTEST} 
      PREFIX ${CMAKE_CURRENT_BINARY_DIR}/3rd-party
      CMAKE_COMMAND cmake -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/3rd-party ..
      GIT_REPOSITORY "https://github.com/google/googletest.git"
      PATCH_COMMAND ""
      UPDATE_COMMAND ""
      INSTALL_COMMAND make install
      )

    ExternalProject_Get_Property(${EXTERNAL_PROJECT_GTEST} install_dir)
    set(GTEST_INCLUDE_DIRS ${install_dir}/include)
    set(GMOCK_INCLUDE_DIRS ${install_dir}/include)
    
    set(GTEST_LIBRARIES
      ${install_dir}/lib/libgtest.so
      )
    set(GTEST_MAIN_LIBRARIES
      ${install_dir}/lib/libgtest_main.so
      )
    set(GTEST_BOTH_LIBRARIES
      ${GTEST_LIBRARIES}
      ${GTEST_MAIN_LIBRARIES}
      )

    set(GMOCK_LIBRARIES
      ${install_dir}/lib/libgmock.so
      )
    set(GMOCK_MAIN_LIBRARIES
      ${install_dir}/lib/libgmock_main.so
      )
    set(GMOCK_BOTH_LIBRARIES
      ${GMOCK_LIBRARIES}
      ${GMOCK_MAIN_LIBRARIES}
      )    
  else()
    ########################################################
    # Find GTest
    ########################################################
    set(EXTERNAL_PROJECT_GTEST "")
    find_package (GTest REQUIRED)
    find_package (GMock REQUIRED)
  endif()
  
  include_directories(${GTEST_INCLUDE_DIRS})
  include_directories(${GMOCK_INCLUDE_DIRS})

  add_subdirectory(test)  
endif()

# Add a distclean target to the Makefile
set (DIST_CLEAN_DIRS "${PROJECT_LIB_DIR};${PROJECT_BIN_DIR};${PROJECT_PLUGIN_LIBS_DIR};${CMAKE_BINARY_DIR}")
ADD_CUSTOM_TARGET(distclean
  COMMAND ${CMAKE_COMMAND} -DDIRS_TO_REMOVE="${DIST_CLEAN_DIRS}" -P ${CMAKE_SOURCE_DIR}/cmake/Modules/distclean.cmake
  )

###############################################################################
# Installation
###############################################################################
set(PROJECT_DESCRIPTION "(>>>PROJECT_NAME<<<) Plugins")

IF(UNIX AND NOT APPLE)
  # - Add each library to LIBS_STRING
  # SCRIMMAGE_GTRI_SIM_LIBS is a "global" variable that is passed
  # down to each sub directory and every library that is
  # generated in this project appends its name to this
  # variable.
  foreach(I ${PROJECT_LIBS})
    set (LIBS_STRING "${LIBS_STRING} -l${I}")
    #message("=======> ${I}")
  endforeach()

  # Add all library targets to the build-tree export set
  export(TARGETS ${PROJECT_LIBS} ${PROJECT_PLUGINS}
    FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake")

  # Export the package for use from the build-tree
  # (this registers the build-tree with a global CMake-registry ~/.cmake)
  export(PACKAGE ${PROJECT_NAME})
  
  # Generate the cmake config file for the source/build tree
  set(PROJECT_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/include
    ${MSGS_INCLUDE_DIR}
    ${PROTO_INCLUDE_DIR}
    )
  
  # Generate the config file that sets *_INCLUDE_DIRS and *_LIBRARIES
  configure_file(${CMAKE_SOURCE_DIR}/cmake/Modules/project-config.cmake.in
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config.cmake" @ONLY)

  # Generate the cmake config file for the install tree
  set(PROJECT_INCLUDE_DIRS
    ${CMAKE_INSTALL_PREFIX}/include
    )
  
  # Generate the config file that sets *_INCLUDE_DIRS and *_LIBRARIES
  configure_file(${CMAKE_SOURCE_DIR}/cmake/Modules/project-config.cmake.in
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}-config.cmake" @ONLY)

  # Generate the package config file for the install tree
  configure_file (
    "${PROJECT_MODULES_DIR}/project.pc.in"
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}.pc"
    @ONLY)

  # Generate the cmake version file for both build/source and install trees
  configure_file (
    "${PROJECT_MODULES_DIR}/project-config-version.cmake.in"
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
    @ONLY
    )   

  # Install all plugin XML files under etc
  install(
    DIRECTORY plugins
    DESTINATION etc/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.xml"
    )

  # Install all plugin header files under include/PROJECT_NAME
  install(
    DIRECTORY plugins
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.h"
    )

  # Install all library headers
  install(
    DIRECTORY include/${PROJECT_NAME}
    DESTINATION include
    PATTERN "*.pyc" EXCLUDE
    PATTERN "*__pycache__*" EXCLUDE
    )

  # Install all project proto headers  
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
  
  ## Install the export set for use with the install-tree
  ## Each library needs to export to this same export target name
  #install(
  #  EXPORT ${PROJECT_NAME}-targets       
  #  DESTINATION lib/cmake/${PROJECT_NAME}
  #  )

  # Install the cmake config
  install (FILES     
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}-config.cmake"
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME}
    )  
  
  # Install the ${PROJECT_NAME}.pc file to lib/pkgconfig
  install (FILES 
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}.pc"    
    DESTINATION lib/pkgconfig
    )

ENDIF()

###############################################################################
# Binary installation generation
###############################################################################
set(VERSION "0.0.1")
set(CPACK_PACKAGE_VERSION ${VERSION})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${PROJECT_DESCRIPTION}")
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME ${PROJECT_NAME})
set(CPACK_PACKAGE_RELEASE 1)
set(CPACK_PACKAGE_CONTACT "Author Name")
set(CPACK_PACKAGE_VENDOR "Organization")
set(CPACK_PACKAGING_INSTALL_PREFIX "/usr")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_PACKAGE_RELEASE}.${CMAKE_SYSTEM_PROCESSOR}")
include(CPack)
