# - Try to find Scrimmage
# Once done, this will define
#
#  SCRIMMAGE_FOUND - system has Scrimmage
#  SCRIMMAGE_INCLUDE_DIRS - the Scrimmage include directories
#  SCRIMMAGE_LIBRARIES - link these to use Scrimmage

include(LibFindMacros)

# Dependencies (forward, required or quietly)
#libfind_package(scrimmage_SIM scrimmage)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(SCRIMMAGE_PKGCONF Scrimmage)

# Include dir
find_path(SCRIMMAGE_INCLUDE_DIR
  NAMES scrimmage/simcontrol/SimControl.h
  PATHS ${SCRIMMAGE_PKGCONF_INCLUDE_DIRS}
  )

#message("==================================================")
#message("SCRIMMAGE_INCLUDE_DIR: ${SCRIMMAGE_INCLUDE_DIR}")
#message("SCRIMMAGE_PKGCONF_LIBRARY_DIRS: ${SCRIMMAGE_PKGCONF_LIBRARY_DIRS}")
#message("SCRIMMAGE_PKGCONF_INCLUDE_DIRS : ${SCRIMMAGE_PKGCONF_INCLUDE_DIRS}")
#message("==================================================")

# Find all the relevant scrimmage libraries
find_library(SCRIMMAGE_PROTOS_LIBRARY
  NAMES scrimmage-protos
  PATHS ${SCRIMMAGE_PKGCONF_LIBRARY_DIRS}
  )

find_library(SCRIMMAGE_AUTONOMY_LIBRARY
  NAMES scrimmage-autonomy
  PATHS ${SCRIMMAGE_PKGCONF_LIBRARY_DIRS}
  )

find_library(SCRIMMAGE_PLUGIN_MANAGER_LIBRARY
  NAMES scrimmage-plugin-manager
  PATHS ${SCRIMMAGE_PKGCONF_LIBRARY_DIRS}
  )

find_library(SCRIMMAGE_LOG_LIBRARY
  NAMES scrimmage-log
  PATHS ${SCRIMMAGE_PKGCONF_LIBRARY_DIRS}
  )

find_library(SCRIMMAGE_MATH_LIBRARY
  NAMES scrimmage-math
  PATHS ${SCRIMMAGE_PKGCONF_LIBRARY_DIRS}
  )

find_library(SCRIMMAGE_COMMON_LIBRARY
  NAMES scrimmage-common
  PATHS ${SCRIMMAGE_PKGCONF_LIBRARY_DIRS}
  )

find_library(MISSIONPARSE_LIBRARY
  NAMES scrimmage-parse
  PATHS ${SCRIMMAGE_PKGCONF_LIBRARY_DIRS}
  )

find_library(SIMCONTROL_LIBRARY
  NAMES scrimmage-simcontrol
  PATHS ${SCRIMMAGE_PKGCONF_LIBRARY_DIRS}
  )

find_library(SCRIMMAGE_METRICS_LIBRARY
  NAMES scrimmage-metrics
  PATHS ${SCRIMMAGE_PKGCONF_LIBRARY_DIRS}
  )

find_library(SCRIMMAGE_VIEWER_LIBRARY
  NAMES scrimmage-viewer
  PATHS ${SCRIMMAGE_PKGCONF_LIBRARY_DIRS}
  )

find_library(SCRIMMAGE_PLUGIN_MESSAGES
  NAMES scrimmage-plugin-messages
  PATHS ${SCRIMMAGE_PKGCONF_LIBRARY_DIRS}
  )

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(SCRIMMAGE_PROCESS_INCLUDES SCRIMMAGE_INCLUDE_DIR)
set(SCRIMMAGE_PROCESS_LIBS
  SCRIMMAGE_PROTOS_LIBRARY
  SCRIMMAGE_AUTONOMY_LIBRARY
  SCRIMMAGE_PLUGIN_MANAGER_LIBRARY
  SCRIMMAGE_LOG_LIBRARY
  SCRIMMAGE_MATH_LIBRARY
  SCRIMMAGE_COMMON_LIBRARY
  MISSIONPARSE_LIBRARY
  SIMCONTROL_LIBRARY
  SCRIMMAGE_METRICS_LIBRARY
  SCRIMMAGE_VIEWER_LIBRARY
  SCRIMMAGE_PLUGIN_MESSAGES
  )

libfind_process(SCRIMMAGE)

#message(----------------------------------)
#message("SCRIMMAGE LIBRARIES:")
#message(${SCRIMMAGE_LIBRARIES})
#message("SCRIMMAGE INCLUDE DIR:")
#message(${SCRIMMAGE_INCLUDE_DIRS})
#message(----------------------------------)
