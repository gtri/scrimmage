set(LIBRARY_NAME scrimmage-xml)
set(LIB_MAJOR 0)
set(LIB_MINOR 0)
set(LIB_RELEASE 0)

set(SRCS 
  RapidXMLParser.cpp
  #  XMLParser.cpp
)

set(INCLUDE_DIRS
  "$<INSTALL_INTERFACE:include>"
  "$<BUILD_INTERFACE:${SCRIMMAGE_INC_DIR}>"
)

if(LibXml2_FOUND)
   set(SRCS ${SRCS} LibXML2Parser.cpp)
   set(PUBLIC_LIBS ${PUBLIC_LIBS} ${LIBXML2_LIBRARIES})
   set(INCLUDE_DIRS ${INCLUDE_DIRS} ${LIBXML2_INCLUDE_DIRS})
   set(DEFINITIONS ${DEFINITIONS} -DENABLE_LIBXML2_PARSER=1)
else()
   set(DEFINITIONS ${DEFINITIONS} -DENABLE_LIBXML2_PARSER=1)
endif()

add_library(${LIBRARY_NAME} 
  SHARED
  ${SRCS}
)

target_link_libraries(${LIBRARY_NAME}
  PUBLIC
  ${PUBLIC_LIBS}
  # TinyXML is a header-only library, so we don't need to link it
)

target_include_directories(${LIBRARY_NAME}
  PUBLIC
  ${INCLUDE_DIRS}
)

set(_soversion ${LIB_MAJOR}.${LIB_MINOR}.${LIB_RELEASE})

set_target_properties(${LIBRARY_NAME} PROPERTIES
  SOVERSION ${LIB_MAJOR}
  VERSION ${_soversion}
  )

install(TARGETS ${LIBRARY_NAME}
  # IMPORTANT: Add the library to the "export-set"
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)

# Push up the PROJECT_LIBS variable
set(PROJECT_LIBS ${PROJECT_LIBS} ${LIBRARY_NAME} PARENT_SCOPE)
