# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_mycar_description_detailed_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED mycar_description_detailed_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(mycar_description_detailed_FOUND FALSE)
  elseif(NOT mycar_description_detailed_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(mycar_description_detailed_FOUND FALSE)
  endif()
  return()
endif()
set(_mycar_description_detailed_CONFIG_INCLUDED TRUE)

# output package information
if(NOT mycar_description_detailed_FIND_QUIETLY)
  message(STATUS "Found mycar_description_detailed: 0.0.0 (${mycar_description_detailed_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'mycar_description_detailed' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${mycar_description_detailed_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(mycar_description_detailed_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${mycar_description_detailed_DIR}/${_extra}")
endforeach()
