add_subdirectory(SimpleCamera)
add_subdirectory(NoisyState)
add_subdirectory(NoisyContacts)
if (OPENCV_FOUND)
    add_subdirectory(ContactBlobCamera)
endif()

set(PROJECT_PLUGINS ${PROJECT_PLUGINS} PARENT_SCOPE)
