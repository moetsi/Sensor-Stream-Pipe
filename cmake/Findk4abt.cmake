#.rst:
# Findk4abt
# ---------
#
# Find Azure Kinect Body Tracking SDK include dirs, and libraries.
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
#
# This module defines the :prop_tgt:`IMPORTED` targets:
#
# ``k4a::k4abt``
#  Defined if the system has Azure Kinect Body Tracking SDK.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
#
# This module sets the following variables:
#
# ::
#
#   k4abt_FOUND               True in case Azure Kinect Body Tracking SDK is found, otherwise false
#   k4abt_ROOT                Path to the root of found Azure Kinect Body Tracking SDK installation
#
# Example Usage
# ^^^^^^^^^^^^^
#
# ::
#
#     find_package(k4abt REQUIRED)
#
#     add_executable(foo foo.cc)
#     target_link_libraries(foo k4a::k4abt)
#
# License
# ^^^^^^^
#
# Copyright (c) 2019 Tsukasa SUGIURA
# Distributed under the MIT License.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#

find_path(k4abt_INCLUDE_DIR
  NAMES
    k4abt.h
  HINTS
    $ENV{K4ABT_ROOT}/sdk/
    /usr/include
  PATHS
    "$ENV{PROGRAMW6432}/Azure Kinect Body Tracking SDK/sdk/"
  PATH_SUFFIXES
    include
)

find_library(k4abt_LIBRARY
  NAMES
    k4abt.lib
    libk4abt.so
  HINTS
    $ENV{K4ABT_ROOT}/sdk/windows-desktop/amd64/release
    /usr/lib
  PATHS
    "$ENV{PROGRAMW6432}/Azure Kinect Body Tracking SDK/sdk/windows-desktop/amd64/release"
  PATH_SUFFIXES
    lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  k4abt DEFAULT_MSG
  k4abt_LIBRARY k4abt_INCLUDE_DIR
)

if(k4abt_FOUND)
  add_library(k4a::k4abt SHARED IMPORTED)
  set_target_properties(k4a::k4abt PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${k4abt_INCLUDE_DIR}")

  set_property(TARGET k4a::k4abt APPEND PROPERTY IMPORTED_CONFIGURATIONS "RELEASE")
  set_target_properties(k4a::k4abt PROPERTIES IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX")
  if(WIN32)
    set_target_properties(k4a::k4abt PROPERTIES IMPORTED_IMPLIB_RELEASE "${k4abt_LIBRARY}")
  else()
    set_target_properties(k4a::k4abt PROPERTIES IMPORTED_LOCATION_RELEASE "${k4abt_LIBRARY}")
  endif()

  set_property(TARGET k4a::k4abt APPEND PROPERTY IMPORTED_CONFIGURATIONS "DEBUG")
  set_target_properties(k4a::k4abt PROPERTIES IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX")
  if(WIN32)
    set_target_properties(k4a::k4abt PROPERTIES IMPORTED_IMPLIB_DEBUG "${k4abt_LIBRARY}")
  else()
    set_target_properties(k4a::k4abt PROPERTIES IMPORTED_LOCATION_DEBUG "${k4abt_LIBRARY}")
  endif()

  get_filename_component(k4abt_ROOT "${k4abt_INCLUDE_DIR}" PATH)
endif()
