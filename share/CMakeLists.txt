add_subdirectory(scrimmage)
if (NOT EXTERNAL AND ${VTK_FOUND})
    add_subdirectory(scrimmage-viz)
    add_subdirectory(scrimmage-playback)
endif()

