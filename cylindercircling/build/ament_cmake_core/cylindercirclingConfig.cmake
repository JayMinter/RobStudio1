# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_cylindercircling_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED cylindercircling_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(cylindercircling_FOUND FALSE)
  elseif(NOT cylindercircling_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(cylindercircling_FOUND FALSE)
  endif()
  return()
endif()
set(_cylindercircling_CONFIG_INCLUDED TRUE)

# output package information
if(NOT cylindercircling_FIND_QUIETLY)
  message(STATUS "Found cylindercircling: 0.0.0 (${cylindercircling_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'cylindercircling' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${cylindercircling_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(cylindercircling_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${cylindercircling_DIR}/${_extra}")
endforeach()
