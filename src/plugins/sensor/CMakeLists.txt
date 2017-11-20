add_subdirectory(SimpleCamera)
add_subdirectory(NoisyState)
add_subdirectory(NoisyContacts)
add_subdirectory(RayTrace)
add_subdirectory(AirSimSensor)
add_subdirectory(ContactBlobCamera)

if (GRPC_FOUND AND NOT EXTERNAL)
  add_subdirectory(TutorialOpenAISensor)
endif()

set(PROJECT_PLUGINS ${PROJECT_PLUGINS} PARENT_SCOPE)
