include(CMakeParseArguments)

function(BuildPPAFromRepo)
  set(options "")
  set(oneValueArgs NAME GIT_REPOSITORY GIT_TAG SOURCE_VERSION PPA PPA_NUMBER GPG_KEY_ID)
  set(multiValueArgs CONFIGURE_COMMAND PATCH_COMMAND UPDATE_COMMAND)
  cmake_parse_arguments(ARG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

  execute_process(COMMAND date -R OUTPUT_VARIABLE DATE_STRING)
  configure_file(${CMAKE_SOURCE_DIR}/packages/${ARG_NAME}/debian/changelog.in
    ${CMAKE_SOURCE_DIR}/packages/${ARG_NAME}/debian/changelog @ONLY)

  ExternalProject_Add(
    ${ARG_NAME}-ppa
    GIT_REPOSITORY ${ARG_GIT_REPOSITORY}
    GIT_TAG        ${ARG_GIT_TAG}
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}
    BUILD_IN_SOURCE 1
    UPDATE_COMMAND "${ARG_UPDATE_COMMAND}"
    PATCH_COMMAND "${ARG_PATCH_COMMAND}"
    CONFIGURE_COMMAND "${ARG_CONFIGURE_COMMAND}"
    CMAKE_COMMAND ""
    BUILD_COMMAND cp -r ${CMAKE_SOURCE_DIR}/packages/${ARG_NAME}/debian ${CMAKE_BINARY_DIR}/src/${ARG_NAME}-ppa/debian
    COMMAND cd ${CMAKE_BINARY_DIR}/src && ${TAR} -acf ${ARG_NAME}_${ARG_SOURCE_VERSION}.${ARG_PPA_NUMBER}.orig.tar.gz ${ARG_NAME}-ppa --exclude-vcs
    COMMAND cd ${CMAKE_BINARY_DIR}/src/${ARG_NAME}-ppa && ${DEBUILD} -i -S -sa -k${ARG_GPG_KEY_ID}
    COMMAND cd ${CMAKE_BINARY_DIR}/src && dput ${ARG_PPA} ${ARG_NAME}_${ARG_SOURCE_VERSION}.${ARG_PPA_NUMBER}-0ppa${ARG_PPA_NUMBER}_source.changes
    INSTALL_COMMAND ""
    #TEST_AFTER_INSTALL rm ${CMAKE_SOURCE_DIR}/debian-config/${ARG_NAME}/changelog
    )
  # Make all ppa projects not build by default
  set_target_properties(${ARG_NAME}-ppa PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)

endfunction()

function(BuildPPA)
  set(options "")
  set(oneValueArgs NAME SOURCE_VERSION PPA PPA_NUMBER GPG_KEY_ID)
  set(multiValueArgs)
  cmake_parse_arguments(ARG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

  execute_process(COMMAND date -R OUTPUT_VARIABLE DATE_STRING)
  configure_file(${CMAKE_SOURCE_DIR}/packages/${ARG_NAME}/debian/changelog.in
    ${CMAKE_SOURCE_DIR}/packages/${ARG_NAME}/debian/changelog @ONLY)

  add_custom_target(${ARG_NAME}-ppa
    COMMAND cp -r ${CMAKE_SOURCE_DIR}/packages/${ARG_NAME} ${CMAKE_BINARY_DIR}/src/${ARG_NAME}-ppa
    COMMAND cd ${CMAKE_BINARY_DIR}/src && ${TAR} -acf ${ARG_NAME}_${ARG_SOURCE_VERSION}.${ARG_PPA_NUMBER}.orig.tar.gz ${ARG_NAME}-ppa --exclude-vcs
    COMMAND cd ${CMAKE_BINARY_DIR}/src/${ARG_NAME}-ppa && ${DEBUILD} -i -S -sa -k${ARG_GPG_KEY_ID}
    COMMAND cd ${CMAKE_BINARY_DIR}/src && dput ${ARG_PPA} ${ARG_NAME}_${ARG_SOURCE_VERSION}.${ARG_PPA_NUMBER}-0ppa${ARG_PPA_NUMBER}_source.changes
    )
  set_target_properties(${ARG_NAME}-ppa PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)

endfunction()
