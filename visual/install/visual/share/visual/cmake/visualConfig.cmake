# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_visual_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED visual_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(visual_FOUND FALSE)
  elseif(NOT visual_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(visual_FOUND FALSE)
  endif()
  return()
endif()
set(_visual_CONFIG_INCLUDED TRUE)

# output package information
if(NOT visual_FIND_QUIETLY)
  message(STATUS "Found visual: 0.0.1 (${visual_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'visual' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${visual_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(visual_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${visual_DIR}/${_extra}")
endforeach()
