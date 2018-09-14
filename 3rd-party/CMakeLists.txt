cmake_minimum_required (VERSION 3.5)

project (scrimmage-deps)

set(PATCH_DIR "${CMAKE_SOURCE_DIR}/patches")
set(SETUP_DIR "${CMAKE_SOURCE_DIR}/setup")
set(PATCH_SCRIPT "${SETUP_DIR}/apply-patch.sh")

option(BUILD_BINARY_PACKAGES "Build deb packages for dependencies." OFF)
option(BUILD_SOURCE_PACKAGES "Build sources packages for dependencies." OFF)

if (BUILD_BINARY_PACKAGES AND BUILD_SOURCE_PACKAGES)
  message(FATAL_ERROR "You can only build source or binary packages, not both at one time.")
endif()

set(TEMP_INSTALL_PREFIX $ENV{HOME}/.local)

if (BUILD_BINARY_PACKAGES)
  # Find the fpm program
  find_program(FPM fpm)
  if (NOT FPM)
    message(FATAL_ERROR
      "Unable to find fpm program for package management."
      "\nPossible fix:"
      "\n\tsudo apt-get install ruby-dev build-essential "
      "\n\tsudo gem install fpm")
  endif()

  set(PACKAGE_OUTPUT_TYPE "deb" CACHE STRING "Package output type (deb, rpm, etc)")

  set(SCRIMMAGE_PREFIX_DIR "opt/scrimmage")
  set(TEMP_INSTALL_PREFIX "/${SCRIMMAGE_PREFIX_DIR}")
  set(DESTDIR "${CMAKE_CURRENT_BINARY_DIR}/installdir")
  set(PROTOBUF_DESTDIR "${DESTDIR}/protobuf")
  set(PYBIND11_DESTDIR "${DESTDIR}/pybind11")
  set(JSBSIM_DESTDIR "${DESTDIR}/jsbsim")
  set(GRPC_INSTALL_PREFIX "${DESTDIR}/grpc/${SCRIMMAGE_PREFIX_DIR}")
  set(JSBSIM_INSTALL_PREFIX "${DESTDIR}/jsbsim/${SCRIMMAGE_PREFIX_DIR}")
  set(SCRIMMAGE_DEPS_LIB_DIR "${PROTOBUF_DESTDIR}${TEMP_INSTALL_PREFIX}/lib:${GRPC_INSTALL_PREFIX}/lib")

elseif(BUILD_SOURCE_PACKAGES)
  set(SCRIMMAGE_PREFIX_DIR "opt/scrimmage")
  set(TEMP_INSTALL_PREFIX "/${SCRIMMAGE_PREFIX_DIR}")
  set(PROTOBUF_DESTDIR "")

else()
  set(PROTOBUF_DESTDIR "")
  set(PYBIND11_DESTDIR "")
  set(JSBSIM_DESTDIR "")
  set(GRPC_INSTALL_PREFIX ${TEMP_INSTALL_PREFIX})
  set(JSBSIM_INSTALL_PREFIX ${TEMP_INSTALL_PREFIX})
  set(SCRIMMAGE_DEPS_LIB_DIR "${TEMP_INSTALL_PREFIX}/lib")
endif()

if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX ${TEMP_INSTALL_PREFIX} CACHE PATH "Change default install path." FORCE)
else()
  set(JSBSIM_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
  set(GRPC_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
endif()

set(SCRIMMAGE_LOCAL_CONFIG_DIR "$ENV{HOME}/.scrimmage")
set(SCRIMMAGE_ENV_DIR "${SCRIMMAGE_LOCAL_CONFIG_DIR}/env")
file(MAKE_DIRECTORY ${SCRIMMAGE_ENV_DIR})
configure_file(${CMAKE_SOURCE_DIR}/cmake/Modules/setenv.in
  "${SCRIMMAGE_ENV_DIR}/${PROJECT_NAME}-setenv" @ONLY)
execute_process(COMMAND bash "-c" "${SETUP_DIR}/edit-config.sh ${SCRIMMAGE_LOCAL_CONFIG_DIR} ${PROJECT_NAME}" OUTPUT_VARIABLE result)

include(ExternalProject)

set(JSBSIM_GIT_REPOSITORY git://git.code.sf.net/p/jsbsim/code)
set(JSBSIM_GIT_TAG 36de9ac63c959cef5d7b2c56fb49c1a57fd46369)
ExternalProject_Add (
  jsbsim
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}
  GIT_REPOSITORY ${JSBSIM_GIT_REPOSITORY}
  GIT_TAG ${JSBSIM_GIT_TAG}
  PATCH_COMMAND ${PATCH_SCRIPT} ${PATCH_DIR}/jsbsim.patch
  UPDATE_COMMAND ${SETUP_DIR}/install-aircraft.sh "${CMAKE_CURRENT_BINARY_DIR}/src/jsbsim"
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${JSBSIM_INSTALL_PREFIX} -DCMAKE_INSTALL_RPATH=\$ORIGIN/../lib -DBUILD_SHARED_LIBS=ON -DINSTALL_JSBSIM_EXEC=ON
  )

########################################################
# Download and build protobuf
########################################################
find_program(ENV_EXECUTABLE env)

set(PROTOBUF_CMAKE_MODULE_DIR ${CMAKE_INSTALL_PREFIX}/share/cmake/protobuf/)
set(PROTOBUF_VERSION_NUM 3.3.0)
set(PROTOBUF_VERSION v${PROTOBUF_VERSION_NUM})
set(PROTOBUF_GIT_REPOSITORY "https://github.com/google/protobuf.git")
ExternalProject_Add (
  protobuf
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}
  GIT_REPOSITORY ${PROTOBUF_GIT_REPOSITORY}
  GIT_TAG ${PROTOBUF_VERSION}
  BUILD_IN_SOURCE 1
  CONFIGURE_COMMAND ./autogen.sh COMMAND ${ENV_EXECUTABLE} LDFLAGS='-Wl,-rpath=\$$ORIGIN/../lib' && ./configure --prefix=${CMAKE_INSTALL_PREFIX}
  PATCH_COMMAND ""
  UPDATE_COMMAND ""
  INSTALL_COMMAND make install DESTDIR="${PROTOBUF_DESTDIR}"
  )

########################################################
# Download and build grpc
########################################################
configure_file(${PATCH_DIR}/grpc.patch.in ${PATCH_DIR}/grpc.patch)
set(GRPC_VERSION_NUM 1.2.1)
set(GRPC_VERSION v${GRPC_VERSION_NUM})
set(GRPC_GIT_REPOSITORY https://github.com/grpc/grpc)
ExternalProject_Add (
  grpc
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}
  BUILD_IN_SOURCE 1
  CONFIGURE_COMMAND ""
  BUILD_COMMAND make HAS_SYSTEM_PROTOBUF=true PROTOC=${PROTOBUF_DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/protoc
  DEPENDS protobuf
  GIT_REPOSITORY ${GRPC_GIT_REPOSITORY}
  GIT_TAG ${GRPC_VERSION}
  UPDATE_COMMAND ""
  PATCH_COMMAND ${PATCH_SCRIPT} ${PATCH_DIR}/grpc.patch
  INSTALL_COMMAND make install prefix=${GRPC_INSTALL_PREFIX}
  #  CMAKE_ARGS
  #    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
  #    -DCMAKE_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}
  #    -DgRPC_PROTOBUF_PROVIDER="package"
  #    -DgRPC_SSL_PROVIDER="package"
  )

########################################################
# Download and build pybind11
########################################################
set(PYBIND11_GIT_REPOSITORY https://github.com/pybind/pybind11.git)
set(PYBIND11_GIT_TAG v2.2.3)
ExternalProject_Add(
  pybind11
  GIT_REPOSITORY    ${PYBIND11_GIT_REPOSITORY}
  GIT_TAG           ${PYBIND11_GIT_TAG}
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}
  CMAKE_ARGS -DPYBIND11_TEST:BOOL=OFF -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
  INSTALL_COMMAND make install DESTDIR=${PYBIND11_DESTDIR}
  )

set(OPENCV3_VERSION 3.4.0)
set(OPENCV3_GIT_REPOSITORY https://github.com/opencv/opencv.git)
set(OPENCV3_CONTRIB_GIT_REPOSITORY https://github.com/opencv/opencv_contrib.git)

set(PROTOBUF_VERSION_NUM 3.3.0)
set(PROTOBUF_VERSION v${PROTOBUF_VERSION_NUM})

set(VTK8_VERSION_NUM 8.1.0)
set(VTK8_GIT_TAG v${VTK8_VERSION_NUM})
set(VTK8_GIT_REPOSITORY https://gitlab.kitware.com/vtk/vtk.git)

if (BUILD_BINARY_PACKAGES)
  add_custom_command(
    TARGET protobuf
    POST_BUILD
    COMMAND ${FPM} --force -s dir -t ${PACKAGE_OUTPUT_TYPE} -n scrimmage-protobuf -v ${PROTOBUF_VERSION_NUM} -C ${DESTDIR}/protobuf -p scrimmage_protobuf_VERSION_ARCH.${PACKAGE_OUTPUT_TYPE} ${SCRIMMAGE_PREFIX_DIR}
    )

  add_custom_command(
    TARGET grpc
    POST_BUILD
    COMMAND ${FPM} --force -s dir -t ${PACKAGE_OUTPUT_TYPE} -n scrimmage-grpc -v ${GRPC_VERSION_NUM} -C ${DESTDIR}/grpc -p scrimmage_grpc_VERSION_ARCH.${PACKAGE_OUTPUT_TYPE} ${SCRIMMAGE_PREFIX_DIR}
    )

  add_custom_command(
    TARGET jsbsim
    POST_BUILD
    COMMAND ${FPM} --force -s dir -t ${PACKAGE_OUTPUT_TYPE} -n scrimmage-jsbsim -v 0.0.1 -C ${DESTDIR}/jsbsim -p scrimmage_jsbsim_VERSION_ARCH.${PACKAGE_OUTPUT_TYPE} ${SCRIMMAGE_PREFIX_DIR}
    )

  add_custom_command(
    TARGET pybind11
    POST_BUILD
    COMMAND ${FPM} --force -s dir -t ${PACKAGE_OUTPUT_TYPE} -n scrimmage-pybind11 -v 0.0.1 -C ${DESTDIR}/pybind11 -p scrimmage_pybind11_VERSION_ARCH.${PACKAGE_OUTPUT_TYPE} ${SCRIMMAGE_PREFIX_DIR}
    )
endif()


if (BUILD_SOURCE_PACKAGES)
  # Only kevin-demarco can push to this PPA. However, if you setup your own PPA
  # on Launchpad and generate the appropriate GPG keys, you can push to your
  # own PPA.
  find_package(CMakeDebSrc)
  SET(PPA "ppa:kevin-demarco/scrimmage" CACHE STRING "Launchpad PPA for dput.")
  SET(GPG_KEY_ID "3951DA01" CACHE STRING "GPG Key ID on local system.")
  SET(DISTRIBUTIONS "xenial jessie" CACHE STRING "Target Linux distributions.")
  SET(ARCHITECTURES "amd64 i386 armhf arm64" CACHE STRING "Possible build architectures.")

  BuildDebSrcFromRepo(
    NAME "scrimmage-pybind11"
    DISTRIBUTIONS ${DISTRIBUTIONS}
    ARCHITECTURES ${ARCHITECTURES}
    PACKAGES_DIR ${CMAKE_SOURCE_DIR}/packages
    GIT_REPOSITORY ${PYBIND11_GIT_REPOSITORY}
    GIT_TAG ${PYBIND11_GIT_TAG}
    SOURCE_VERSION 2.2.3
    PPA_VERSION_NUMBER 6
    PPA ${PPA}
    GPG_KEY_ID ${GPG_KEY_ID}
    UPDATE_COMMAND ""
    PATCH_COMMAND ""
    CONFIGURE_COMMAND ""
    )

  BuildDebSrcFromRepo(
    NAME "scrimmage-protobuf3"
    DISTRIBUTIONS ${DISTRIBUTIONS}
    ARCHITECTURES ${ARCHITECTURES}
    PACKAGES_DIR ${CMAKE_SOURCE_DIR}/packages
    GIT_REPOSITORY ${PROTOBUF_GIT_REPOSITORY}
    GIT_TAG ${PROTOBUF_VERSION}
    SOURCE_VERSION ${PROTOBUF_VERSION_NUM}
    PPA_VERSION_NUMBER 4
    PPA ${PPA}
    GPG_KEY_ID ${GPG_KEY_ID}
    UPDATE_COMMAND ""
    PATCH_COMMAND ""
    CONFIGURE_COMMAND ./autogen.sh
    )

  BuildDebSrcFromRepo(
    NAME "scrimmage-grpc"
    DISTRIBUTIONS ${DISTRIBUTIONS}
    ARCHITECTURES ${ARCHITECTURES}
    PACKAGES_DIR ${CMAKE_SOURCE_DIR}/packages
    GIT_REPOSITORY ${GRPC_GIT_REPOSITORY}
    GIT_TAG ${GRPC_VERSION}
    SOURCE_VERSION ${GRPC_VERSION_NUM}
    PPA_VERSION_NUMBER 7
    PPA ${PPA}
    GPG_KEY_ID ${GPG_KEY_ID}
    UPDATE_COMMAND ""
    PATCH_COMMAND ${PATCH_SCRIPT} ${PATCH_DIR}/grpc.patch
    CONFIGURE_COMMAND ""
    )

  BuildDebSrcFromRepo(
    NAME "scrimmage-jsbsim"
    DISTRIBUTIONS ${DISTRIBUTIONS}
    ARCHITECTURES ${ARCHITECTURES}
    PACKAGES_DIR ${CMAKE_SOURCE_DIR}/packages
    GIT_REPOSITORY ${JSBSIM_GIT_REPOSITORY}
    GIT_TAG ${JSBSIM_GIT_TAG}
    SOURCE_VERSION 0.1
    PPA_VERSION_NUMBER 5
    PPA ${PPA}
    GPG_KEY_ID ${GPG_KEY_ID}
    UPDATE_COMMAND ""
    PATCH_COMMAND ${PATCH_SCRIPT} ${PATCH_DIR}/jsbsim.patch && ${SETUP_DIR}/install-aircraft.sh -c "${CMAKE_CURRENT_BINARY_DIR}/src/scrimmage-jsbsim-xenial-deb-src"
    CONFIGURE_COMMAND ""
    )

  BuildDebSrcFromRepo(
    NAME "scrimmage-opencv3"
    DISTRIBUTIONS ${DISTRIBUTIONS}
    ARCHITECTURES ${ARCHITECTURES}
    PACKAGES_DIR ${CMAKE_SOURCE_DIR}/packages
    GIT_REPOSITORY ${OPENCV3_GIT_REPOSITORY}
    GIT_TAG ${OPENCV3_VERSION}
    SOURCE_VERSION ${OPENCV3_VERSION}
    PPA_VERSION_NUMBER 4
    PPA ${PPA}
    GPG_KEY_ID ${GPG_KEY_ID}
    UPDATE_COMMAND git clone https://github.com/opencv/opencv_contrib.git --branch 3.4.0
    PATCH_COMMAND ""
    CONFIGURE_COMMAND ""
    )

  BuildDebSrcFromRepo(
    NAME "scrimmage-vtk8"
    DISTRIBUTIONS ${DISTRIBUTIONS}
    ARCHITECTURES ${ARCHITECTURES}
    PACKAGES_DIR ${CMAKE_SOURCE_DIR}/packages
    GIT_REPOSITORY ${VTK8_GIT_REPOSITORY}
    GIT_TAG ${VTK8_GIT_TAG}
    SOURCE_VERSION ${VTK8_VERSION_NUM}
    PPA_VERSION_NUMBER 0
    PPA ${PPA}
    GPG_KEY_ID ${GPG_KEY_ID}
    UPDATE_COMMAND ""
    PATCH_COMMAND ""
    CONFIGURE_COMMAND ""
    )

  BuildDebSrcFromRepo(
    NAME "scrimmage"
    DISTRIBUTIONS ${DISTRIBUTIONS}
    ARCHITECTURES ${ARCHITECTURES}
    PACKAGES_DIR ${CMAKE_SOURCE_DIR}/packages
    GIT_REPOSITORY https://github.com/gtri/scrimmage.git
    GIT_TAG master
    SOURCE_VERSION 0.2.0
    PPA_VERSION_NUMBER 0
    PPA ${PPA}
    GPG_KEY_ID ${GPG_KEY_ID}
    UPDATE_COMMAND ""
    PATCH_COMMAND ""
    CONFIGURE_COMMAND ""
    )

  BuildDebSrcFromDir(
    NAME "scrimmage-dependencies"
    DISTRIBUTIONS ${DISTRIBUTIONS}
    ARCHITECTURES ${ARCHITECTURES}
    PACKAGES_DIR ${CMAKE_SOURCE_DIR}/packages
    PROJECT_DIRECTORY ${CMAKE_SOURCE_DIR}/packages/scrimmage-dependencies/scrimmage-dependencies
    SOURCE_VERSION 0.2.1
    PPA ${PPA}
    PPA_VERSION_NUMBER 6
    GPG_KEY_ID ${GPG_KEY_ID}
    )

  BuildDebSrcFromRepo(
    NAME "scrimmage-extra-data"
    DISTRIBUTIONS ${DISTRIBUTIONS}
    ARCHITECTURES ${ARCHITECTURES}
    PACKAGES_DIR ${CMAKE_SOURCE_DIR}/packages
    GIT_REPOSITORY https://github.com/SyllogismRXS/scrimmage-extra-data.git
    GIT_TAG master
    SOURCE_VERSION 0.1
    PPA_VERSION_NUMBER 6
    PPA ${PPA}
    GPG_KEY_ID ${GPG_KEY_ID}
    UPDATE_COMMAND ""
    PATCH_COMMAND ""
    CONFIGURE_COMMAND ""
    )

endif()
