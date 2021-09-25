#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "lzma::lzma" for configuration "Release"
set_property(TARGET lzma::lzma APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(lzma::lzma PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/liblzma.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS lzma::lzma )
list(APPEND _IMPORT_CHECK_FILES_FOR_lzma::lzma "${_IMPORT_PREFIX}/lib/liblzma.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
