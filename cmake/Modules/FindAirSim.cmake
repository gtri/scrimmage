set(AIRSIM_ROOT_SEARCH "" CACHE STRING "The path to AirSim's source code.")
if ("${AIRSIM_ROOT_SEARCH}" STREQUAL "")
  set(AIRSIM_FOUND FALSE)
endif()

find_path(AIRSIM_SOURCE_ROOT
  NAMES "./AirLib/include/vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
  PATHS ${AIRSIM_ROOT_SEARCH}
  DOC "Base directory of the AirSim git repository"
  NO_DEFAULT_PATH
  )

if ("${AIRSIM_SOURCE_ROOT}" STREQUAL "AIRSIM_SOURCE_ROOT-NOTFOUND" OR
    "${AIRSIM_SOURCE_ROOT}" STREQUAL "")
  set(AIRSIM_FOUND FALSE)
  set(AIRSIM_INCLUDE_DIRS "AIRSIM_INCLUDE_DIRS-NOTFOUND")
  set(AIRSIM_LIBRARIES "AIRSIM_LIBRARIES-NOTFOUND")
else()
  set(AIRSIM_FOUND TRUE)
  set(AIRSIM_INCLUDE_DIRS ${AIRSIM_SOURCE_ROOT}/AirLib/include ${AIRSIM_SOURCE_ROOT}/external/rpclib/rpclib-2.2.1/include)
  set(AIRSIM_LIBRARIES "${AIRSIM_SOURCE_ROOT}/AirLib/lib/libAirLib.a"
    "${AIRSIM_SOURCE_ROOT}/AirLib/deps/rpclib/lib/librpc.a")
  # set(AIRSIM_LIBRARIES "${AIRSIM_SOURCE_ROOT}/cmake/output/lib/libAirLib.a"
  #   "${AIRSIM_SOURCE_ROOT}/cmake/output/lib/librpc.a")
endif()
