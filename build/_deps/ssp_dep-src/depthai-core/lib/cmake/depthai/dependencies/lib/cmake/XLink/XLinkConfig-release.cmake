#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "XLink" for configuration "Release"
set_property(TARGET XLink APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(XLink PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libXLink.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS XLink )
list(APPEND _IMPORT_CHECK_FILES_FOR_XLink "${_IMPORT_PREFIX}/lib/libXLink.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
