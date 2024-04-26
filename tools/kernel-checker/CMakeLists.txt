set (APP_NAME kernel-checker)

file (GLOB SRCS *.cpp)
file (GLOB HDRS *.h)

add_executable(${APP_NAME} ${SRCS})

target_link_libraries(${APP_NAME}
  OpenCL::OpenCL)

target_compile_definitions(${APP_NAME} 
  PUBLIC
  CL_HPP_MINIMUM_OPENCL_VERSION=120
  CL_HPP_TARGET_OPENCL_VERSION=120)

