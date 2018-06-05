# This CMake script will delete build directories and files to bring the
# package back to its distribution state

# Loop over the directories and delete each one
string(REPLACE " " ";" DIRS_TO_REMOVE ${DIRS_TO_REMOVE})
FOREACH(D ${DIRS_TO_REMOVE})  
  IF(EXISTS ${D})    
    FILE(REMOVE_RECURSE ${D})
  ENDIF()
ENDFOREACH()

