###############################################################################
# Sphinx setup
###############################################################################
find_python_module(sphinx_git PY_SPHINX_GIT_FOUND REQUIRED)
if (NOT PY_SPHINX_GIT_FOUND)
  message(WARNING "Can't find sphinx_git Python module.")
  return()
endif()

find_package(Sphinx)

if(NOT DEFINED SPHINX_THEME)
  set(SPHINX_THEME "sphinx_rtd_theme")
endif()

if(NOT DEFINED SPHINX_THEME_DIR)
  set(SPHINX_THEME_DIR "sphinx_rtd_theme.get_html_theme_path()")
endif()

set(SPHINX_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/source")

# configured documentation tools and intermediate build results
set(BINARY_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/_build")

# Sphinx cache with pickled ReST documents
set(SPHINX_CACHE_DIR "${CMAKE_CURRENT_BINARY_DIR}/_doctrees")

# HTML output directory
set(SPHINX_HTML_DIR "${CMAKE_CURRENT_BINARY_DIR}/sphinx/html")

configure_file(
  "${SPHINX_SOURCE_DIR}/conf.py.in"
  "${SPHINX_SOURCE_DIR}/conf.py"
  @ONLY)

add_custom_target(sphinx-docs
  ${SPHINX_EXECUTABLE}
  -q -b html
  -c "${SPHINX_SOURCE_DIR}"
  -d "${SPHINX_CACHE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/source"
  "${SPHINX_HTML_DIR}"
  COMMENT "Building HTML documentation with Sphinx")

###############################################################################
# Jekyll Website Build
###############################################################################
find_program(BUNDLE bundle)
if(BUNDLE_NOTFOUND)
  message("Failed to find bundle / gem. Not building jekyll website.")
else()
  add_custom_target(website-docs
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/jekyll
    COMMAND bundle install
    COMMAND JEKYLL_ENV=production bundle exec jekyll build --destination ${PROJECT_BINARY_DIR}/docs/jekyll
    COMMENT "Building scrimmage website."
    )
endif()

###############################################################################
# Doxygen setup
###############################################################################
find_package(Doxygen)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doxyfile.in
  ${CMAKE_CURRENT_BINARY_DIR}/doxyfile @ONLY)

add_custom_target(doxygen-docs
  COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/bibtex.bib ${CMAKE_CURRENT_BINARY_DIR} && ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/doxyfile
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Generating API documentation with Doxygen"
  VERBATIM
  )

###############################################################################
# Create custom target that builds both sphinx and doxygen targets
###############################################################################
add_custom_target(docs COMMENT "Building Sphinx and Doxygen documentation.")
add_dependencies(docs sphinx-docs doxygen-docs)

# if (NOT BUNDLE_NOTFOUND)
#   add_dependencies(docs website-docs)
# endif()
