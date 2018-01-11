# - Try to find  JSBSIM
# Once done, this will define
#
#  JSBSIM_FOUND - system has scicoslab
#  JSBSIM_INCLUDE_DIRS - the scicoslab include directories
#  JSBSIM_LIBRARIES - libraries to link to

include(LibFindMacros)
include(MacroCommonPaths)

MacroCommonPaths(JSBSIM JSBSim)

## Include dir
#find_path(JSBSIM_INCLUDE_DIR
#    NAMES JSBSim/FGFDMExec.h
#    PATHS ${COMMON_INCLUDE_PATHS_JSBSIM}
#)
# Include dir

find_path(JSBSIM_INCLUDE_DIR
    NAMES FGFDMExec.h
    PATHS ${COMMON_INCLUDE_PATHS_JSBSIM}
    NO_DEFAULT_PATH
    )

if (NOT JSBSIM_INCLUDE_DIR)
    find_path(JSBSIM_INCLUDE_DIR
        NAMES JSBSim/FGFDMExec.h
        PATHS ${COMMON_INCLUDE_PATHS_JSBSIM}
    )
endif()

# data dir
find_path(JSBSIM_DATA_DIR_SEARCH
    NAMES JSBSim/aircraft/aircraft_template.xml
    PATHS ${COMMON_DATA_PATHS_JSBSIM}
)
set(JSBSIM_DATA_DIR ${JSBSIM_DATA_DIR_SEARCH}/JSBSim)

# Finally the library itself
find_library(JSBSIM_LIBRARY
    NAMES JSBSim
    PATHS ${COMMON_LIBRARY_PATHS_JSBSIM}
)

# message("+++++++++++++++++++++++++++++++++++++++++++++++++++++++")
# message("${COMMON_INCLUDE_PATHS_JSBSIM}")
# message("+++++++++++++++++++++++++++++++++++++++++++++++++++++++")
# message("${JSBSIM_INCLUDE_DIR}")
# message("+++++++++++++++++++++++++++++++++++++++++++++++++++++++")
# message("${COMMON_LIBRARY_PATHS_JSBSIM}")
# message("+++++++++++++++++++++++++++++++++++++++++++++++++++++++")
# message("${JSBSIM_LIBRARY}")
# message("+++++++++++++++++++++++++++++++++++++++++++++++++++++++")

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(JSBSIM_PROCESS_INCLUDES JSBSIM_INCLUDE_DIR)
set(JSBSIM_PROCESS_LIBS JSBSIM_LIBRARY JSBSIM_LIBRARIES)
set(JSBSIM_INCLUDE_DIR ${JSBSIM_INCLUDE_DIR} ${JSBSIM_INCLUDE_DIR}/JSBSim)
set(JSBSIM_INCLUDES ${JSBSIM_INCLUDES} ${JSBSIM_INCLUDE_DIR}/JSBSim)

libfind_process(JSBSIM)
macro(find_or_build_jsbsim TAG EP_BASE_DIR EP_INSTALL_PREFIX EP_DATADIR)
    find_package(JSBSIM ${TAG})
    if(NOT JSBSIM_FOUND)
        ExternalProject_Add(jsbsim
            GIT_REPOSITORY "git://github.com/jgoppert/jsbsim.git"
            GIT_TAG ${TAG}
            UPDATE_COMMAND ""
            CONFIGURE_COMMAND ${EP_BASE_DIR}/Source/jsbsim/autogen.sh --enable-libraries --prefix=${EP_INSTALL_PREFIX}
            BUILD_COMMAND make -j4
            BUILD_IN_SOURCE 1
            INSTALL_DIR ${EP_BASE_DIR}/${EP_INSTALL_PREFIX}
            CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${EP_INSTALL_PREFIX}
            INSTALL_COMMAND make DESTDIR=${EP_BASE_DIR} install
            )
        set(JSBSIM_INCLUDE_DIRS  ${EP_BASE_DIR}/${EP_INSTALL_PREFIX}/include ${EP_BASE_DIR}/${EP_INSTALL_PREFIX}/include/JSBSim)
        set(JSBSIM_DATA_DIR ${EP_DATADIR}/jsbsim)
         # static lib prefix
        if(WIN32)
            set(STATIC_LIB_PREFIX "")
        elseif(APPLE)
            set(STATIC_LIB_PREFIX "lib")
        elseif(UNIX)
            set(STATIC_LIB_PREFIX "lib")
        else()
            message(FATAL_ERROR "unknown operating system")
        endif()

        set(JSBSIM_LIBRARIES ${EP_LIBDIR}/${STATIC_LIB_PREFIX}JSBSim.a)
        set(JSBSIM_FOUND TRUE)
    endif()
endmacro()
