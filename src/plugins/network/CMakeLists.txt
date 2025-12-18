add_subdirectory(GlobalNetwork)
add_subdirectory(LocalNetwork)
add_subdirectory(SphereNetwork)

if(ENABLE_GPU_ACCELERATION)
  add_subdirectory(GPUSphereNetwork)
endif()

set(PROJECT_PLUGINS ${PROJECT_PLUGINS} PARENT_SCOPE)
