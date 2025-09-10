# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_wind_pyrosens_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED wind_pyrosens_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(wind_pyrosens_FOUND FALSE)
  elseif(NOT wind_pyrosens_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(wind_pyrosens_FOUND FALSE)
  endif()
  return()
endif()
set(_wind_pyrosens_CONFIG_INCLUDED TRUE)

# output package information
if(NOT wind_pyrosens_FIND_QUIETLY)
  message(STATUS "Found wind_pyrosens: 0.0.1 (${wind_pyrosens_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'wind_pyrosens' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${wind_pyrosens_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(wind_pyrosens_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${wind_pyrosens_DIR}/${_extra}")
endforeach()
