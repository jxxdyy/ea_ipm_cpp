# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ea_ipm_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ea_ipm_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ea_ipm_FOUND FALSE)
  elseif(NOT ea_ipm_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ea_ipm_FOUND FALSE)
  endif()
  return()
endif()
set(_ea_ipm_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ea_ipm_FIND_QUIETLY)
  message(STATUS "Found ea_ipm: 0.0.0 (${ea_ipm_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ea_ipm' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${ea_ipm_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ea_ipm_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${ea_ipm_DIR}/${_extra}")
endforeach()
