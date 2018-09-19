###############################################
#      Protocol Buffer Code Generation        #
###############################################
# A single shared library is created for all
# protocol buffer files

# PROTO input - finds all .proto files in the current dir
file(GLOB PROTO_FILE_LIST
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")

set(GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/msgs)
file(MAKE_DIRECTORY ${GEN_DIR})

foreach(PROTO_FILE ${PROTO_FILE_LIST})
  # Get the name of the file without extension
  # name.proto -> name

  get_filename_component(PROTO_NAME ${PROTO_FILE} NAME_WE)
  get_filename_component(ABS_PROTO_NAME ${PROTO_FILE} REALPATH)
  get_filename_component(PROTO_PATH ${ABS_PROTO_NAME} DIRECTORY)
  get_filename_component(SCRIMMAGE_PROTO_PATH ${SCRIMMAGE_PROTO_PATH} REALPATH)

  set(PROTO_FILE_GEN_CPP_FILES
    ${GEN_DIR}/${PROTO_NAME}.pb.cc
    )
  set(PROTO_FILE_GEN_H_FILES
    ${GEN_DIR}/${PROTO_NAME}.pb.h
    )

  if (GRPC_FOUND)
    set(PROTO_FILE_GEN_CPP_FILES
      ${PROTO_FILE_GEN_CPP_FILES}
      ${GEN_DIR}/${PROTO_NAME}.grpc.pb.cc
      )
    set(PROTO_FILE_GEN_H_FILES
      ${PROTO_FILE_GEN_H_FILES}
      ${GEN_DIR}/${PROTO_NAME}.grpc.pb.h
      )
  endif()

  set(PROTO_FILE_GEN_FILES
    ${PROTO_FILE_GEN_CPP_FILES}
    ${PROTO_FILE_GEN_H_FILES}
    )

  if (GRPC_FOUND)
    add_custom_command(
      OUTPUT ${PROTO_FILE_GEN_FILES}
      COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} ${ABS_PROTO_NAME} --proto_path=${PROTO_PATH} --proto_path=${SCRIMMAGE_PROTO_PATH} --cpp_out=${GEN_DIR} --grpc_out=${GEN_DIR} --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN}
      COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} ${ABS_PROTO_NAME} --proto_path=${PROTO_PATH} --proto_path=${SCRIMMAGE_PROTO_PATH} --python_out=${GEN_DIR} --grpc_python_out=${GEN_DIR} --plugin=protoc-gen-grpc_python=${GRPC_PYTHON_PLUGIN}
      DEPENDS ${PROTO_FILE}
      WORKING_DIRECTORY ${PROTO_PATH}
      )
  else()
    add_custom_command(
      OUTPUT ${PROTO_FILE_GEN_FILES}
      COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} ${ABS_PROTO_NAME} --proto_path=${PROTO_PATH} --proto_path=${SCRIMMAGE_PROTO_PATH} --cpp_out=${GEN_DIR} --python_out=${GEN_DIR}
      DEPENDS ${PROTO_FILE}
      WORKING_DIRECTORY ${PROTO_PATH}
      )
  endif()

 # Make a list of all generated CPP files
 set(GEN_CPP_FILES
   ${GEN_CPP_FILES}
   ${PROTO_FILE_GEN_CPP_FILES}
   )

  # Make a list of all generated H files
  set(GEN_H_FILES
    ${GEN_H_FILES}
    ${PROTO_FILE_GEN_H_FILES}
    )
endforeach()

##############################################
#          Library Creation                  #
##############################################
SET (LIBRARY_NAME ${PROJECT_NAME}-msgs)
SET (LIB_MAJOR 0)
SET (LIB_MINOR 0)
SET (LIB_RELEASE 1)

ADD_LIBRARY( ${LIBRARY_NAME} SHARED
  ${GEN_CPP_FILES}
  ${GEN_H_FILES}
  )

add_dependencies(${PROJECT_NAME}-msgs scrimmage-protos)

target_link_libraries(${LIBRARY_NAME}
  ${PROTOBUF_LIBRARIES}
  )

SET (_soversion ${LIB_MAJOR}.${LIB_MINOR}.${LIB_RELEASE})

set_target_properties(
  ${LIBRARY_NAME}
  PROPERTIES
  SOVERSION ${LIB_MAJOR}
  VERSION ${_soversion}
  )

target_include_directories(${LIBRARY_NAME}
  PUBLIC
    "$<INSTALL_INTERFACE:include;${PROTOBUF_INCLUDE_DIRS}>"
    "$<BUILD_INTERFACE:${MSGS_INCLUDE_DIR};${PROTOBUF_INCLUDE_DIRS}>"
)

install(TARGETS ${LIBRARY_NAME}
  # IMPORTANT: Add the library to the "export-set"
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)

set(PROJECT_LIBS ${PROJECT_LIBS} ${LIBRARY_NAME} PARENT_SCOPE)
