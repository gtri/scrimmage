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

project((>>>PROJECT_NAME<<<))

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
get_filename_component(PROJECT_LIB_DIR ${PROJECT_BINARY_DIR}/lib ABSOLUTE)
get_filename_component(PROJECT_PLUGIN_LIBS_DIR ${PROJECT_BINARY_DIR}/plugin_libs ABSOLUTE)
get_filename_component(PROJECT_BIN_DIR ${PROJECT_BINARY_DIR}/bin ABSOLUTE)
get_filename_component(PROJECT_INC_DIR ./include ABSOLUTE)
get_filename_component(PROJECT_SRC_DIR ./src ABSOLUTE)
get_filename_component(PROJECT_SHARE_DIR ./share ABSOLUTE)
get_filename_component(PROJECT_SCRIPTS_DIR ./scripts ABSOLUTE)
get_filename_component(PROJECT_SETUP_DIR ./setup ABSOLUTE)
get_filename_component(PROJECT_MODULES_DIR ./cmake/Modules ABSOLUTE)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_MODULES_DIR})

include(MacroSubDirList)
include(MacroAddExternalTarget)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING
    "Choose the type of build, options are: Debug Release
      RelWithDebInfo MinSizeRel." FORCE)
endif()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BIN_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_LIB_DIR})

## set the cmake defaults for libraries and binaries
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_LIB_DIR} CACHE PATH
  "Output directory for the dynamic libraries" )
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BIN_DIR} CACHE PATH
  "Output directory for the binaries" )
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_LIB_DIR} CACHE PATH
  "Output directory for the static libraries (archives)" )

set(MSGS_INCLUDE_DIR ${PROJECT_BINARY_DIR}/msgs)
set(PROTO_INCLUDE_DIR ${PROJECT_BINARY_DIR}/src/proto)

###############################################################################
# Find Scrimmage
###############################################################################
find_package(scrimmage REQUIRED)

option(SETUP_LOCAL_CONFIG_DIR "Setup ~/.scrimmage" ON)
include(GenerateSetEnv)
GenerateSetEnv(
  SETUP_LOCAL_CONFIG_DIR ${SETUP_LOCAL_CONFIG_DIR}
  SETENV_IN_FILE ${SCRIMMAGE_CMAKE_MODULES}/setenv.in
  MISSION_PATH ${PROJECT_SOURCE_DIR}/missions
  PLUGIN_PATH ${PROJECT_BINARY_DIR}/plugin_libs
              ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/plugins
  #CONFIG_PATH ${PROJECT_SOURCE_DIR}/config
  )

########################################################
# Recurse into src, share, and plugins
########################################################
#add_subdirectory(share)
add_subdirectory(msgs)
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
    ############################################################
    # https://github.com/google/googletest/tree/master/googletest#incorporating-into-an-existing-cmake-project
    ############################################################

    # Download and unpack googletest at configure time
    if (NOT TARGET gtest_main)
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/CMakeLists.txt.gtest.in
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

        # The gtest/gtest_main targets carry header search path
        # dependencies automatically when using CMake 2.8.11 or
        # later. Otherwise we have to add them here ourselves.
        if (CMAKE_VERSION VERSION_LESS 2.8.11)
          include_directories("${gtest_SOURCE_DIR}/include")
        endif()
      endif()
  endif()

  add_subdirectory(test)
endif()

# Add a distclean target to the Makefile
set (DIST_CLEAN_DIRS "${PROJECT_LIB_DIR};${PROJECT_BIN_DIR};${PROJECT_PLUGIN_LIBS_DIR};${CMAKE_BINARY_DIR}")
ADD_CUSTOM_TARGET(distclean
  COMMAND ${CMAKE_COMMAND} -DDIRS_TO_REMOVE="${DIST_CLEAN_DIRS}" -P ${PROJECT_SOURCE_DIR}/cmake/Modules/distclean.cmake
  )

###############################################################################
# Installation
###############################################################################
set(PROJECT_DESCRIPTION "(>>>PROJECT_NAME<<<) Plugins")

if(UNIX AND NOT APPLE)
  # - Add each library to LIBS_STRING
  # SCRIMMAGE_GTRI_SIM_LIBS is a "global" variable that is passed
  # down to each sub directory and every library that is
  # generated in this project appends its name to this
  # variable.
  foreach(I ${PROJECT_LIBS})
    set (LIBS_STRING "${LIBS_STRING} -l${I}")
    #message("=======> ${I}")
  endforeach()

  #############################################################################
  # Generate the cmake configuration files for the build tree
  #############################################################################
  include(CMakePackageConfigHelpers)

  set(INCLUDE_INSTALL_DIR include/ CACHE PATH "Include install dir")
  set(LIB_INSTALL_DIR lib/ CACHE PATH "Library install dir")
  set(CMAKE_CONFIG_DEST "${LIB_INSTALL_DIR}/${PROJECT_NAME}/cmake")

  configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/project-config.cmake.in
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
    INSTALL_DESTINATION ${PROJECT_BINARY_DIR}
    PATH_VARS INCLUDE_INSTALL_DIR
    )

  write_basic_package_version_file(
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
    )

  # Add all library targets to the build-tree export set
  export(TARGETS ${PROJECT_LIBS} ${PROJECT_PLUGINS}
    FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake")

  # Export the package for use from the build-tree
  # (this registers the build-tree with a global CMake-registry ~/.cmake)
  export(PACKAGE ${PROJECT_NAME})

  #############################################################################
  # Generate the cmake configuration files for the install tree
  #############################################################################
  configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/project-config.cmake.in
    ${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}-config.cmake
    INSTALL_DESTINATION ${CMAKE_CONFIG_DEST}
    PATH_VARS INCLUDE_INSTALL_DIR
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

  # install export targets
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

  if (NOT EXTERNAL)
    # Install meshes, terrain, maps
    install(
      DIRECTORY data/gui
      DESTINATION share/${PROJECT_NAME}/data
      )
  endif()

  # Create the ld.so.conf.d/project.conf file
  file(WRITE
    ${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}.conf
    ${CMAKE_INSTALL_PREFIX}/lib
    )
  install(
    FILES ${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}.conf
    DESTINATION /etc/ld.so.conf.d
    )

endif()

###############################################################################
# Binary installation generation
###############################################################################
set(CPACK_INSTALL_CMAKE_PROJECTS "${PROJECT_BINARY_DIR};${PROJECT_NAME};ALL;/")
set(CPACK_OUTPUT_CONFIG_FILE "${PROJECT_BINARY_DIR}/CPackConfig.cmake")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${PROJECT_DESCRIPTION}")
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME ${PROJECT_NAME})
set(CPACK_PACKAGE_RELEASE 1)
set(CPACK_PACKAGE_CONTACT "Author Name")
set(CPACK_PACKAGE_VENDOR "Organization")
set(CPACK_PACKAGING_INSTALL_PREFIX "/usr")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_PACKAGE_RELEASE}.${CMAKE_SYSTEM_PROCESSOR}")
include(CPack)
