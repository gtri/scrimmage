SubDirList(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR})

if (NOT ENABLE_PYTHON_BINDINGS)
    list(REMOVE_ITEM SUBDIRS ScrimmageOpenAISensor)
    list(REMOVE_ITEM SUBDIRS RLSimpleSensor)
    list(REMOVE_ITEM SUBDIRS RLConsensusSensor)
endif()

FOREACH(subdir ${SUBDIRS})
  ADD_SUBDIRECTORY(${subdir})
ENDFOREACH()

set(PROJECT_PLUGINS ${PROJECT_PLUGINS} PARENT_SCOPE)
