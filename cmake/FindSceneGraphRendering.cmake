
# This is FindSceneGraphRendering.cmake
# CMake module to locate the SceneGraphRendering package
#
# The following cache variables may be set before calling this script:
#
# SceneGraphRendering_DIR (or SceneGraphRendering_ROOT): (Optional) The install prefix OR source tree of SceneGraphRendering (e.g. /usr/local or src/SceneGraphRendering)
# SceneGraphRendering_BUILD_NAME:          (Optional) If compiling against a source tree, the name of the build directory
#                            within it (e.g build-debug).  Without this defined, this script tries to
#                            intelligently find the build directory based on the project's build directory name
#                            or based on the build type (Debug/Release/etc).
#
# The following variables will be defined:
#
# SceneGraphRendering_FOUND          : TRUE if the package has been successfully found
# SceneGraphRendering_INCLUDE_DIR    : paths to SceneGraphRendering's INCLUDE directories
# SceneGraphRendering_LIBS           : paths to SceneGraphRendering's libraries
#
# NOTES on compiling against an uninstalled SceneGraphRendering build tree:
# - A SceneGraphRendering source tree will be automatically searched for in the directory
#   'SceneGraphRendering' next to your project directory, after searching
#   CMAKE_INSTALL_PREFIX and $HOME, but before searching /usr/local and /usr.
# - The build directory will be searched first with the same name as your
#   project's build directory, e.g. if you build from 'MyProject/build-optimized',
#   'SceneGraphRendering/build-optimized' will be searched first.  Next, a build directory for
#   your project's build type, e.g. if CMAKE_BUILD_TYPE in your project is
#   'Release', then 'SceneGraphRendering/build-release' will be searched next.  Finally, plain
#   'SceneGraphRendering/build' will be searched.
# - You can control the SceneGraphRendering build directory name directly by defining the CMake
#   cache variable 'SceneGraphRendering_BUILD_NAME', then only 'SceneGraphRendering/${SceneGraphRendering_BUILD_NAME} will
#   be searched.
# - Use the standard CMAKE_PREFIX_PATH, or SceneGraphRendering_DIR, to find a specific SceneGraphRendering
#   directory.

# Get path suffixes to help look for SceneGraphRendering
if(SceneGraphRendering_BUILD_NAME)
  set(SceneGraphRendering_build_names "${SceneGraphRendering_BUILD_NAME}/SceneGraphRendering")
else()
  # lowercase build type
  string(TOLOWER "${CMAKE_BUILD_TYPE}" build_type_suffix)
  # build suffix of this project
  get_filename_component(my_build_name "${CMAKE_BINARY_DIR}" NAME)
  
  set(SceneGraphRendering_build_names "${my_build_name}/SceneGraphRendering" "build-${build_type_suffix}/SceneGraphRendering" "build/SceneGraphRendering")
endif()

# Use SceneGraphRendering_ROOT or SceneGraphRendering_DIR equivalently
if(SceneGraphRendering_ROOT AND NOT SceneGraphRendering_DIR)
  set(SceneGraphRendering_DIR "${SceneGraphRendering_ROOT}")
endif()

if(SceneGraphRendering_DIR)
  # Find include dirs
  find_path(SceneGraphRendering_INCLUDE_DIR SceneGraphRendering/SceneGraphRendering.h
    PATHS "${SceneGraphRendering_DIR}/include" "${SceneGraphRendering_DIR}" NO_DEFAULT_PATH
    DOC "SceneGraphRendering include directories")

  # Find libraries
  find_library(SceneGraphRendering_LIBS NAMES SceneGraphRendering
    HINTS "${SceneGraphRendering_DIR}/lib" "${SceneGraphRendering_DIR}" NO_DEFAULT_PATH
    PATH_SUFFIXES ${SceneGraphRendering_build_names}
    DOC "SceneGraphRendering libraries")
else()
  # Find include dirs
  set(extra_include_paths ${CMAKE_INSTALL_PREFIX}/include "$ENV{HOME}/include" "${PROJECT_SOURCE_DIR}/../SceneGraphRendering" /usr/local/include /usr/include)
  find_path(SceneGraphRendering_INCLUDE_DIR SceneGraphRendering/SceneGraphRendering.h
    PATHS ${extra_include_paths}
    DOC "SceneGraphRendering include directories")
  if(NOT SceneGraphRendering_INCLUDE_DIR)
    message(STATUS "Searched for SceneGraphRendering headers in default paths plus ${extra_include_paths}")
  endif()

  # Find libraries
  find_library(SceneGraphRendering_LIBS NAMES SceneGraphRendering
    HINTS ${CMAKE_INSTALL_PREFIX}/lib "$ENV{HOME}/lib" "${PROJECT_SOURCE_DIR}/../SceneGraphRendering" /usr/local/lib /usr/lib
    PATH_SUFFIXES ${SceneGraphRendering_build_names}
    DOC "SceneGraphRendering libraries")
endif()

# handle the QUIETLY and REQUIRED arguments and set SceneGraphRendering_FOUND to TRUE
# if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SceneGraphRendering DEFAULT_MSG
                                  SceneGraphRendering_LIBS SceneGraphRendering_INCLUDE_DIR)
 
