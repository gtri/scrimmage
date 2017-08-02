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

find_library(SCRIMMAGE_LIBRARY
  NAMES scrimmage
  PATHS ${SCRIMMAGE_PKGCONF_LIBRARY_DIRS}
)

find_library(SCRIMMAGE_VIEWER_LIBRARY
  NAMES scrimmage-viewer
  PATHS ${SCRIMMAGE_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(SCRIMMAGE_PROCESS_INCLUDES SCRIMMAGE_INCLUDE_DIR)
set(SCRIMMAGE_PROCESS_LIBS
  SCRIMMAGE_LIBRARY
  SCRIMMAGE_PROTOS_LIBRARY
  SCRIMMAGE_VIEWER_LIBRARY
  )

libfind_process(SCRIMMAGE)
