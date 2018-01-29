function(ConfigureDebianDir)
  # Copy debian directory into source tree
  file(COPY ${SRC_DIR}/ DESTINATION ${DEST_DIR})

  # Get the date string
  execute_process(COMMAND date -R OUTPUT_VARIABLE DATE_STRING)

  # Update the changelog file with cmake variables
  configure_file(${DEST_DIR}/debian/changelog.in ${DEST_DIR}/debian/changelog @ONLY)

  # Clean up the input file
  file(REMOVE ${DEST_DIR}/debian/changelog.in)
endfunction()

ConfigureDebianDir()
