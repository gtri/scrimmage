cmake_minimum_required (VERSION 3.5)
project (scrimmage-dependencies)

install(
  PROGRAMS ./scripts/setup.sh
  DESTINATION ${CMAKE_INSTALL_PREFIX}
  )
