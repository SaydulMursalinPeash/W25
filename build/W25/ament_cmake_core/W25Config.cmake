# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_W25_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED W25_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(W25_FOUND FALSE)
  elseif(NOT W25_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(W25_FOUND FALSE)
  endif()
  return()
endif()
set(_W25_CONFIG_INCLUDED TRUE)

# output package information
if(NOT W25_FIND_QUIETLY)
  message(STATUS "Found W25: 0.0.0 (${W25_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'W25' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${W25_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(W25_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${W25_DIR}/${_extra}")
endforeach()
