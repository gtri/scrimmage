# Find if a Python module is installed
# Found at http://www.cmake.org/pipermail/cmake/2011-January/041666.html
# Reference: https://github.com/ivansafrin/Polycode/blob/master/CMake/FindPythonModule.cmake
#
# Example usage:
# find_python_module(sphinx_git PY_SPHINX_GIT_FOUND REQUIRED)
# if (NOT PY_SPHINX_GIT_FOUND)
#   message(WARNING "Can't find sphinx_git Python module.")
#   return()
# endif()
#
# Note: If this function finds a python module and then you remove it, you will
# have to remove the CMakeCache.txt in order to search for the module again. We
# cache the found module to speed up subsequent cmake calls.
#
function(find_python_module module found)
  string(TOUPPER ${module} module_upper)

  # If the module was already found in the cache, return.
  if (PY_${module_upper})
    set(${found} TRUE PARENT_SCOPE)
    return()
  endif()

  if(NOT PY_${module_upper})
	if(ARGC GREATER 2 AND ARGV2 STREQUAL "REQUIRED")
	  set(${module}_FIND_REQUIRED TRUE)
	endif()
	# A module's location is usually a directory, but for binary modules
	# it's a .so file.
	execute_process(COMMAND "${PYTHON_EXECUTABLE}" "-c"
	  "import re, ${module}; print(re.compile('/__init__.py.*').sub('',${module}.__file__))"
	  RESULT_VARIABLE _${module}_status
	  OUTPUT_VARIABLE _${module}_location
	  ERROR_QUIET
	  OUTPUT_STRIP_TRAILING_WHITESPACE
      )

    if(NOT _${module}_status)
      set(MODULE_FOUND "TRUE")
	  set(PY_${module_upper} ${_${module}_location} CACHE STRING
		"Location of Python module ${module}")
	endif()
  endif()
  find_package_handle_standard_args(PY_${module_upper}
    FOUND_VAR PY_${module_upper}_FOUND
    REQUIRED_VARS MODULE_FOUND
    FAIL_MESSAGE "Failed to find python module: ${module}"
    )
  # Set the user-defined found variable
  if (PY_${module_upper}_FOUND)
    set(${found} TRUE PARENT_SCOPE)
  endif()
endfunction()
