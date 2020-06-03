if (ENABLE_PYTHON_BINDINGS)
  add_subdirectory(scrimmage/bindings_src)
endif()

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/setup.py.in
  ${CMAKE_CURRENT_SOURCE_DIR}/setup.py
  )
