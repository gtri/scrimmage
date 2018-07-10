#--------------------------------------------------------
# Library Creation
#--------------------------------------------------------
set(LIBRARY_NAME "py_bindings")

include_directories(include)

pybind11_add_module(${LIBRARY_NAME}
  src/py_bindings.cpp
  src/py_math.cpp
  src/py_pubsub.cpp
  src/py_common.cpp
  src/py_autonomy.cpp
  src/py_openai_env.cpp
  src/py_utils.cpp)

add_dependencies(${LIBRARY_NAME} scrimmage-core)

target_link_libraries(${LIBRARY_NAME} PRIVATE
  ScrimmageOpenAIAutonomy_plugin
  ScrimmageOpenAISensor_plugin
  scrimmage-core)

# Push up the SCRIMMAGE_LIBS variable
set_target_properties(${LIBRARY_NAME} PROPERTIES
  SUFFIX ".so"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  OUTPUT_NAME py_bindings)
