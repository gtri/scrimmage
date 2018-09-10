cmake_minimum_required (VERSION 3.5)
project (scrimmage-dependencies)

configure_file(${CMAKE_CURRENT_LIST_DIR}/scripts/setup.sh.in
  ${CMAKE_CURRENT_BINARY_DIR}/setup.sh @ONLY)

configure_file(${CMAKE_CURRENT_LIST_DIR}/scripts/scrimmage-dependencies.conf.in
  ${CMAKE_CURRENT_BINARY_DIR}/scrimmage-dependencies-${DEB_HOST_MULTIARCH}.conf @ONLY)

install(
  PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/setup.sh
  DESTINATION ${CMAKE_INSTALL_PREFIX}
  )

install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/scrimmage-dependencies-${DEB_HOST_MULTIARCH}.conf
  DESTINATION /etc/ld.so.conf.d
  )
