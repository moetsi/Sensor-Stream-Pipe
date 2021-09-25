#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "zdepth::zstd" for configuration "Release"
set_property(TARGET zdepth::zstd APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(zdepth::zstd PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libzstd.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS zdepth::zstd )
list(APPEND _IMPORT_CHECK_FILES_FOR_zdepth::zstd "${_IMPORT_PREFIX}/lib/libzstd.a" )

# Import target "zdepth::zdepth" for configuration "Release"
set_property(TARGET zdepth::zdepth APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(zdepth::zdepth PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libzdepth.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS zdepth::zdepth )
list(APPEND _IMPORT_CHECK_FILES_FOR_zdepth::zdepth "${_IMPORT_PREFIX}/lib/libzdepth.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
