cmake_minimum_required (VERSION 3.5)
project (scrimmage-dependencies)

configure_file(${CMAKE_CURRENT_LIST_DIR}/scripts/setup.sh.in
  ${CMAKE_CURRENT_BINARY_DIR}/setup.sh @ONLY)

install(
  PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/setup.sh
  DESTINATION ${CMAKE_INSTALL_PREFIX}
  )
