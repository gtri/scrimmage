###############################################
#      Protocol Buffer Code Generation        #
###############################################
# A single shared library is created for all
# protocol buffer files

# Operations are done in subdirectories scrimmage/proto so that 
# protoc generates python files with the correct imports.
# See https://github.com/google/protobuf/issues/1491 for more information.

# PROTO input - finds all .proto files in the current dir
file(GLOB PROTO_FILE_LIST
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/scrimmage/proto/*.proto")

set(GEN_DIR ${CMAKE_CURRENT_BINARY_DIR})
file(MAKE_DIRECTORY ${GEN_DIR}/scrimmage/proto)

if (NOT GRPC_FOUND)
  list(REMOVE_ITEM PROTO_FILE_LIST "Scrimmage.proto")
endif()

foreach(PROTO_FILE ${PROTO_FILE_LIST})    
  # Get the name of the file without extension
  # name.proto -> name
  get_filename_component(PROTO_NAME ${PROTO_FILE} NAME_WE)
  get_filename_component(ABS_PROTO_NAME ${PROTO_FILE} REALPATH)

  set(PROTO_FILE_GEN_CPP_FILES
    ${GEN_DIR}/scrimmage/proto/${PROTO_NAME}.pb.cc    
    )
  set(PROTO_FILE_GEN_H_FILES
    ${GEN_DIR}/scrimmage/proto/${PROTO_NAME}.pb.h
    )

  if (GRPC_FOUND)
    set(PROTO_FILE_GEN_CPP_FILES
      ${PROTO_FILE_GEN_CPP_FILES}
      ${GEN_DIR}/scrimmage/proto/${PROTO_NAME}.grpc.pb.cc      
      )
    set(PROTO_FILE_GEN_H_FILES
      ${PROTO_FILE_GEN_H_FILES}
      ${GEN_DIR}/scrimmage/proto/${PROTO_NAME}.grpc.pb.h
      )        
  endif()
  
  set(PROTO_FILE_GEN_FILES
    ${PROTO_FILE_GEN_CPP_FILES}
    ${PROTO_FILE_GEN_H_FILES}
    )

  if (GRPC_FOUND)
    add_custom_command(
      OUTPUT ${PROTO_FILE_GEN_FILES}
      COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} ${ABS_PROTO_NAME} --proto_path=${CMAKE_CURRENT_SOURCE_DIR} --cpp_out=${GEN_DIR} --grpc_out=${GEN_DIR} --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN}
      COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} ${ABS_PROTO_NAME} --proto_path=${CMAKE_CURRENT_SOURCE_DIR} --python_out=${SCRIMMAGE_PYTHON_DIR} --grpc_python_out=${SCRIMMAGE_PYTHON_DIR} --plugin=protoc-gen-grpc_python=${GRPC_PYTHON_PLUGIN}
      DEPENDS ${ABS_PROTO_NAME} ${EXTERNAL_PROJECT_PROTOBUF} ${EXTERNAL_PROJECT_GRPC}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}      
      )    
  else()
    add_custom_command(
      OUTPUT ${PROTO_FILE_GEN_FILES}
      COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} ${ABS_PROTO_NAME} --proto_path=${CMAKE_CURRENT_SOURCE_DIR} --cpp_out=${GEN_DIR} --python_out=${SCRIMMAGE_PYTHON_DIR}
      DEPENDS ${PROTO_FILE} ${EXTERNAL_PROJECT_PROTOBUF}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
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

set_source_files_properties(${GEN_CPP_FILES}
                            PROPERTIES GENERATED TRUE)
set_source_files_properties(${GEN_H_FILES}
                            PROPERTIES GENERATED TRUE)

set(GEN_TARGET scrimmage-proto-gen)


add_custom_target(
    ${GEN_TARGET}
    DEPENDS ${GEN_CPP_FILES} ${GEN_H_FILES}
    )

##############################################
#          Library Creation                  #
##############################################
SET (LIBRARY_NAME scrimmage-protos)
SET (LIB_MAJOR 0)
SET (LIB_MINOR 0)
SET (LIB_RELEASE 1)

ADD_LIBRARY( ${LIBRARY_NAME} SHARED
  ${GEN_CPP_FILES}
  )

add_dependencies(${LIBRARY_NAME} scrimmage-proto-gen)

AddExternalTarget(${LIBRARY_NAME} EXTERNAL_TARGET_PROTOBUF)
target_link_libraries(${LIBRARY_NAME}
  ${PROTOBUF_LIBRARIES}
  )
if (GRPC_FOUND)
  AddExternalTarget(${LIBRARY_NAME} EXTERNAL_PROJECT_GRPC)
  target_link_libraries(${LIBRARY_NAME}
    ${GRPC_LIBRARIES}
    ) 
endif()

SET (_soversion ${LIB_MAJOR}.${LIB_MINOR}.${LIB_RELEASE})

set_target_properties(
  ${LIBRARY_NAME}
  PROPERTIES
  SOVERSION ${LIB_MAJOR}
  VERSION ${_soversion}
  )

install(TARGETS ${LIBRARY_NAME}
  # IMPORTANT: Add the library to the "export-set"
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  )

# Push up the SCRIMMAGE_SIM_LIBS variable
set(PROJECT_LIBS ${PROJECT_LIBS} ${LIBRARY_NAME} PARENT_SCOPE)
