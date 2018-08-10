include(CMakeParseArguments)

function(GenerateSetEnv)
  set(options)
  set(oneValueArgs SETUP_LOCAL_CONFIG_DIR LOCAL_CONFIG_DIR SETENV_IN_FILE)
  set(multiValueArgs MISSION_PATH PLUGIN_PATH PATH CONFIG_PATH DATA_PATH PYTHONPATH)
  cmake_parse_arguments(ARG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

  if (ARG_SETUP_LOCAL_CONFIG_DIR)
    if ("${ARG_LOCAL_CONFIG_DIR}" STREQUAL "")
      set(ARG_LOCAL_CONFIG_DIR "$ENV{HOME}/.scrimmage")
    endif()

    set(SCRIMMAGE_ENV_DIR "${ARG_LOCAL_CONFIG_DIR}/env")
    set(LOCAL_SETUP_BASH ${ARG_LOCAL_CONFIG_DIR}/setup.bash)
    file(MAKE_DIRECTORY ${SCRIMMAGE_ENV_DIR})

    # Convert CMake lists into file paths separated by ":"
    string(REPLACE ";" ":" ARG_MISSION_PATH "${ARG_MISSION_PATH}")
    if (NOT "${ARG_MISSION_PATH}" STREQUAL "")
      string(CONCAT ARG_MISSION_PATH ":" "${ARG_MISSION_PATH}")
    endif()

    string(REPLACE ";" ":" ARG_PATH "${ARG_PATH}")
    if (NOT "${ARG_PATH}" STREQUAL "")
      string(CONCAT ARG_PATH ":" "${ARG_PATH}")
    endif()

    string(REPLACE ";" ":" ARG_PLUGIN_PATH "${ARG_PLUGIN_PATH}")
    if (NOT "${ARG_PLUGIN_PATH}" STREQUAL "")
      string(CONCAT ARG_PLUGIN_PATH ":" "${ARG_PLUGIN_PATH}")
    endif()

    string(REPLACE ";" ":" ARG_CONFIG_PATH "${ARG_CONFIG_PATH}")
    if (NOT "${ARG_CONFIG_PATH}" STREQUAL "")
      string(CONCAT ARG_CONFIG_PATH ":" "${ARG_CONFIG_PATH}")
    endif()

    string(REPLACE ";" ":" ARG_DATA_PATH "${ARG_DATA_PATH}")
    if (NOT "${ARG_DATA_PATH}" STREQUAL "")
      string(CONCAT ARG_DATA_PATH ":" "${ARG_DATA_PATH}")
    endif()

    string(REPLACE ";" ":" ARG_PYTHONPATH "${ARG_PYTHONPATH}")
    if (NOT "${ARG_PYTHONPATH}" STREQUAL "")
      string(CONCAT ARG_PYTHONPATH ":" "${ARG_PYTHONPATH}")
    endif()

    # Write the project-setenv file
    configure_file(${ARG_SETENV_IN_FILE}
      "${SCRIMMAGE_ENV_DIR}/${PROJECT_NAME}-setenv" @ONLY)

    # Create the ~/.scrimmage/setup.bash file if it doesn't exist
    if(NOT EXISTS ${LOCAL_SETUP_BASH})
      file(WRITE ${LOCAL_SETUP_BASH} "")
    endif()

    if(EXISTS ${LOCAL_SETUP_BASH})
      # Determine if this project's source line exists already
      set(SOURCE_LINE "${SCRIMMAGE_ENV_DIR}/${PROJECT_NAME}-setenv")

      FILE(READ ${LOCAL_SETUP_BASH} FILE_TEXT)
      STRING(FIND "${FILE_TEXT}" "${SOURCE_LINE}" STR_MATCHES)
      if (${STR_MATCHES} EQUAL -1)
        # Append the source line if it doesn't exist
        file(APPEND ${LOCAL_SETUP_BASH} "source ${SOURCE_LINE}\n" )
      endif()
    else()
      message(WARNING "${PROJECT_NAME}'s local configuration directory doesn't exist")
      message(WARNING "${LOCAL_SETUP_BASH}")
    endif()
  endif()

  # Install Tree Paths
  if (NOT "${ARG_MISSION_PATH}" STREQUAL "")
    set(ARG_MISSION_PATH ":${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/missions")
  endif()

  if (NOT "${ARG_PLUGIN_PATH}" STREQUAL "")
    set(ARG_PLUGIN_PATH ":${CMAKE_INSTALL_PREFIX}/etc/${PROJECT_NAME}/plugins:${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/plugin_libs")
  endif()

  if (NOT "${ARG_CONFIG_PATH}" STREQUAL "")
    set(ARG_CONFIG_PATH ":${CMAKE_INSTALL_PREFIX}/etc/${PROJECT_NAME}/config")
  endif()

  if (NOT "${ARG_DATA_PATH}" STREQUAL "")
    set(ARG_DATA_PATH ":${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/data")
  endif()

  if (NOT "${ARG_PATH}" STREQUAL "")
    set(ARG_PATH ":${CMAKE_INSTALL_PREFIX}/bin")
  endif()

  set(ARG_PYTHONPATH "")

  configure_file(${ARG_SETENV_IN_FILE}
    ${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}-setenv @ONLY
    )
endfunction()
