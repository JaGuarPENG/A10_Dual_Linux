# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_dualarm_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED dualarm_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(dualarm_FOUND FALSE)
  elseif(NOT dualarm_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(dualarm_FOUND FALSE)
  endif()
  return()
endif()
set(_dualarm_CONFIG_INCLUDED TRUE)

# output package information
if(NOT dualarm_FIND_QUIETLY)
  message(STATUS "Found dualarm: 0.0.0 (${dualarm_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'dualarm' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${dualarm_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(dualarm_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${dualarm_DIR}/${_extra}")
endforeach()
