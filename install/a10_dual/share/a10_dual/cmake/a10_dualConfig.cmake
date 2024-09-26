# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_a10_dual_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED a10_dual_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(a10_dual_FOUND FALSE)
  elseif(NOT a10_dual_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(a10_dual_FOUND FALSE)
  endif()
  return()
endif()
set(_a10_dual_CONFIG_INCLUDED TRUE)

# output package information
if(NOT a10_dual_FIND_QUIETLY)
  message(STATUS "Found a10_dual: 0.0.0 (${a10_dual_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'a10_dual' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${a10_dual_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(a10_dual_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${a10_dual_DIR}/${_extra}")
endforeach()
