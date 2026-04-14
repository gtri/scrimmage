add_subdirectory(scrimmage)
if (NOT EXTERNAL AND SCRIMMAGE_HAVE_VTK_VIEWER)
    add_subdirectory(scrimmage-viz)
    add_subdirectory(scrimmage-playback)
endif()

