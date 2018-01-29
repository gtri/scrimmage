include(CMakeParseArguments)

function(BuildPPAFromRepo)
  set(options "")
  set(oneValueArgs NAME GIT_REPOSITORY GIT_TAG SOURCE_VERSION PPA PPA_NUMBER GPG_KEY_ID)
  set(multiValueArgs CONFIGURE_COMMAND PATCH_COMMAND UPDATE_COMMAND)
  cmake_parse_arguments(ARG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

  set(DEB_SRC_DIR ${ARG_NAME}-deb-src)

  # Use ExternalProject_Add to clone the source repository, checkout, and any
  # update or patch
  ExternalProject_Add(
    ${DEB_SRC_DIR}
    GIT_REPOSITORY ${ARG_GIT_REPOSITORY}
    GIT_TAG        ${ARG_GIT_TAG}
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}
    BUILD_IN_SOURCE 1
    UPDATE_COMMAND "${ARG_UPDATE_COMMAND}"
    PATCH_COMMAND "${ARG_PATCH_COMMAND}"
    CONFIGURE_COMMAND "${ARG_CONFIGURE_COMMAND}"
    CMAKE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    )

  # Run a custom cmake script that copies the debian directory into the
  # respective source tree and configure the changelog file.
  add_custom_command(
    DEPENDS ${DEB_SRC_DIR} ${CMAKE_SOURCE_DIR}/packages/${ARG_NAME}/debian
    COMMAND ${CMAKE_COMMAND}
    -DARG_SOURCE_VERSION=${ARG_SOURCE_VERSION}
    -DARG_PPA_NUMBER=${ARG_PPA_NUMBER}
    -DSRC_DIR=${CMAKE_SOURCE_DIR}/packages/${ARG_NAME}
    -DDEST_DIR=${CMAKE_BINARY_DIR}/src/${DEB_SRC_DIR}
    -P ${CMAKE_SOURCE_DIR}/cmake/Modules/ConfigureDebianDir.cmake
    OUTPUT ${CMAKE_BINARY_DIR}/src/${DEB_SRC_DIR}/debian
    )

  # Generate the debian source package
  add_custom_target(
    ${ARG_NAME}-debuild
    DEPENDS ${CMAKE_BINARY_DIR}/src/${DEB_SRC_DIR}/debian
    COMMAND cd ${CMAKE_BINARY_DIR}/src && ${TAR} -acf ${ARG_NAME}_${ARG_SOURCE_VERSION}.${ARG_PPA_NUMBER}.orig.tar.gz ${DEB_SRC_DIR} --exclude-vcs
    COMMAND cd ${CMAKE_BINARY_DIR}/src/${DEB_SRC_DIR} && ${DEBUILD} -i -S -sa -k${ARG_GPG_KEY_ID}
    )

  add_custom_target(
    ${ARG_NAME}-local-test
    DEPENDS ${ARG_NAME}-debuild
    COMMAND cd ${CMAKE_BINARY_DIR}/src && pbuilder-dist xenial build ${ARG_NAME}_${ARG_SOURCE_VERSION}.${ARG_PPA_NUMBER}-0ppa${ARG_PPA_NUMBER}.dsc
    )

  # Upload the debian source package to the Launchpad PPA
  add_custom_target(
    ${ARG_NAME}-upload-ppa
    DEPENDS ${ARG_NAME}-debuild
    COMMAND cd ${CMAKE_BINARY_DIR}/src && dput ${ARG_PPA} ${ARG_NAME}_${ARG_SOURCE_VERSION}.${ARG_PPA_NUMBER}-0ppa${ARG_PPA_NUMBER}_source.changes
    )

  # Make all ppa projects not build by default
  set_target_properties(${ARG_NAME}-upload-ppa PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)

endfunction()

function(BuildPPA)
  set(options "")
  set(oneValueArgs NAME SOURCE_VERSION PPA PPA_NUMBER GPG_KEY_ID)
  set(multiValueArgs)
  cmake_parse_arguments(ARG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

  set(DEB_SRC_DIR ${ARG_NAME}-deb-src)

  # Run a custom cmake script that copies the debian directory into the
  # respective source tree and configure the changelog file.
  add_custom_command(
    DEPENDS ${CMAKE_SOURCE_DIR}/packages/${ARG_NAME}/debian
    COMMAND ${CMAKE_COMMAND}
    -DARG_SOURCE_VERSION=${ARG_SOURCE_VERSION}
    -DARG_PPA_NUMBER=${ARG_PPA_NUMBER}
    -DSRC_DIR=${CMAKE_SOURCE_DIR}/packages/${ARG_NAME}
    -DDEST_DIR=${CMAKE_BINARY_DIR}/src/${DEB_SRC_DIR}
    -P ${CMAKE_SOURCE_DIR}/cmake/Modules/ConfigureDebianDir.cmake
    OUTPUT ${CMAKE_BINARY_DIR}/src/${DEB_SRC_DIR}/debian
    )

  # Generate the debian source package
  add_custom_target(
    ${ARG_NAME}-debuild
    DEPENDS ${CMAKE_BINARY_DIR}/src/${DEB_SRC_DIR}/debian
    COMMAND cd ${CMAKE_BINARY_DIR}/src && ${TAR} -acf ${ARG_NAME}_${ARG_SOURCE_VERSION}.${ARG_PPA_NUMBER}.orig.tar.gz ${DEB_SRC_DIR} --exclude-vcs
    COMMAND cd ${CMAKE_BINARY_DIR}/src/${DEB_SRC_DIR} && ${DEBUILD} -i -S -sa -k${ARG_GPG_KEY_ID}
    )

  add_custom_target(
    ${ARG_NAME}-local-test
    DEPENDS ${ARG_NAME}-debuild
    COMMAND cd ${CMAKE_BINARY_DIR}/src && pbuilder-dist xenial build ${ARG_NAME}_${ARG_SOURCE_VERSION}.${ARG_PPA_NUMBER}-0ppa${ARG_PPA_NUMBER}.dsc
    )

  # Upload the debian source package to the Launchpad PPA
  add_custom_target(
    ${ARG_NAME}-upload-ppa
    DEPENDS ${ARG_NAME}-debuild
    COMMAND cd ${CMAKE_BINARY_DIR}/src && dput ${ARG_PPA} ${ARG_NAME}_${ARG_SOURCE_VERSION}.${ARG_PPA_NUMBER}-0ppa${ARG_PPA_NUMBER}_source.changes
    )

  # Make all ppa projects not build by default
  set_target_properties(${ARG_NAME}-upload-ppa PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)

endfunction()
