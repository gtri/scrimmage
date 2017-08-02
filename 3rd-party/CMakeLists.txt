cmake_minimum_required (VERSION 3.5)

project (scrimmage-deps)

set(PATCH_DIR "${CMAKE_SOURCE_DIR}/patches")
set(SETUP_DIR "${CMAKE_SOURCE_DIR}/setup")
set(PATCH_SCRIPT "${SETUP_DIR}/apply-patch.sh")

if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX $ENV{HOME}/.local CACHE PATH "Change default install path." FORCE)
endif()

set(SCRIMMAGE_LOCAL_CONFIG_DIR "$ENV{HOME}/.scrimmage")
set(SCRIMMAGE_ENV_DIR "${SCRIMMAGE_LOCAL_CONFIG_DIR}/env")
file(MAKE_DIRECTORY ${SCRIMMAGE_ENV_DIR})
configure_file(${CMAKE_SOURCE_DIR}/cmake/Modules/setenv.in
  "${SCRIMMAGE_ENV_DIR}/${PROJECT_NAME}-setenv" @ONLY)
execute_process(COMMAND bash "-c" "${SETUP_DIR}/edit-config.sh ${SCRIMMAGE_LOCAL_CONFIG_DIR} ${PROJECT_NAME}" OUTPUT_VARIABLE result)

include(ExternalProject)

ExternalProject_Add (
    jsbsim
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}
    GIT_REPOSITORY "git://git.code.sf.net/p/jsbsim/code"
    GIT_TAG "36de9ac63c959cef5d7b2c56fb49c1a57fd46369"
    PATCH_COMMAND ${PATCH_SCRIPT} ${PATCH_DIR}/jsbsim.patch
    UPDATE_COMMAND ${SETUP_DIR}/install-aircraft.sh "${CMAKE_CURRENT_BINARY_DIR}/src/jsbsim"
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)

########################################################
# Download and build protobuf
########################################################
set(PROTOBUF_CMAKE_MODULE_DIR ${CMAKE_INSTALL_PREFIX}/share/cmake/protobuf/)
set(PROTOBUF_VERSION v3.3.0)
ExternalProject_Add (
  protobuf
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}
  GIT_REPOSITORY "https://github.com/google/protobuf.git"
  GIT_TAG ${Protobuf_VERSION}
  BUILD_IN_SOURCE 1
  CONFIGURE_COMMAND ./autogen.sh COMMAND ./configure --prefix=${CMAKE_INSTALL_PREFIX}
  PATCH_COMMAND ""
  UPDATE_COMMAND ""  
  INSTALL_COMMAND make install
)

#set(CMAKE_PREFIX_PATH $ENV{HOME}/.local/)

configure_file(${PATCH_DIR}/grpc.patch.in ${PATCH_DIR}/grpc.patch)

########################################################
# Download and build grpc
########################################################
set(GRPC_VERSION v1.2.1)
ExternalProject_Add (
  grpc
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}
  BUILD_IN_SOURCE 1
  CONFIGURE_COMMAND ""
  BUILD_COMMAND make HAS_SYSTEM_PROTOBUF=true PROTOC=${CMAKE_INSTALL_PREFIX}/bin/protoc
  DEPENDS protobuf
  GIT_REPOSITORY "https://github.com/grpc/grpc"  
  GIT_TAG ${GRPC_VERSION}
  UPDATE_COMMAND ""
  PATCH_COMMAND ${PATCH_SCRIPT} ${PATCH_DIR}/grpc.patch
  INSTALL_COMMAND make install prefix=${CMAKE_INSTALL_PREFIX}
#  CMAKE_ARGS
#    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
#    -DCMAKE_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}
#    -DgRPC_PROTOBUF_PROVIDER="package"
#    -DgRPC_SSL_PROVIDER="package"
)

ExternalProject_Add(
  pybind11
  GIT_REPOSITORY    https://github.com/pybind/pybind11.git
  GIT_TAG           13d8cd2cc7566de34d724f428ea7a6b6448d6a0c
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}
  CMAKE_ARGS -DPYBIND11_TEST:BOOL=OFF -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)
