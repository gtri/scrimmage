add_subdirectory(autonomy)
add_subdirectory(sensor)
add_subdirectory(motion)
add_subdirectory(controller)
add_subdirectory(metrics)
add_subdirectory(interaction)
add_subdirectory(network)

set(PROJECT_PLUGINS ${PROJECT_PLUGINS} PARENT_SCOPE)
