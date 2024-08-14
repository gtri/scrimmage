set(LIBRARY_NAME scrimmage-gpu)

set(SRCS
  GPUController.cpp
  OpenCLUtils.cpp
  GPUMotionModel.cpp
)

set(INCLUDE_DIRS
  "$<INSTALL_INTERFACE:include>"
  "$<BUILD_INTERFACE:${SCRIMMAGE_INC_DIR}>"
)

add_library(${LIBRARY_NAME}
  SHARED
  ${SRCS}
) 

target_link_libraries(${LIBRARY_NAME}
  PUBLIC
  PRIVATE
  OpenCL::OpenCL
)

target_include_directories(${LIBRARY_NAME}
  PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${INCLUDE_DIRS}
)


install(TARGETS ${LIBRARY_NAME}
  # IMPORTANT: Add the library to the "export-set"
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/${PROJECT_NAME}/libs
)
