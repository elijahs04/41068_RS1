# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_pyrosens_heat_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED pyrosens_heat_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(pyrosens_heat_FOUND FALSE)
  elseif(NOT pyrosens_heat_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(pyrosens_heat_FOUND FALSE)
  endif()
  return()
endif()
set(_pyrosens_heat_CONFIG_INCLUDED TRUE)

# output package information
if(NOT pyrosens_heat_FIND_QUIETLY)
  message(STATUS "Found pyrosens_heat: 0.0.1 (${pyrosens_heat_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'pyrosens_heat' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${pyrosens_heat_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(pyrosens_heat_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${pyrosens_heat_DIR}/${_extra}")
endforeach()
