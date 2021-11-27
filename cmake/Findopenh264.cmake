set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)

find_path(openh264_INCLUDE_DIR
    NAMES codec_api.h
    PATHS include/wels
)
find_library(openh264_LIBRARY
    NAMES libopenh264.so.6
    PATHS lib
)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(openh264
    DEFAULT_MSG
    openh264_INCLUDE_DIR openh264_LIBRARY
)

if(openh264_FOUND)
    set(openh264_LIBRARIES ${openh264_LIBRARY})
    set(openh264_INCLUDE_DIRS ${openh264_INCLUDE_DIR})
endif()

if(openh264_FOUND AND NOT TARGET openh264::openh264)
    add_library(openh264::openh264 UNKNOWN IMPORTED)
    set_target_properties(openh264::openh264 PROPERTIES
        IMPORTED_LOCATION "${openh264_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${openh264_INCLUDE_DIR}"
    )
endif()

mark_as_advanced(
    openh264_INCLUDE_DIR openh264_INCLUDE_DIRS
    openh264_LIBRARY openh264_LIBRARIES)
