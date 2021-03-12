set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)

find_path(NvPipe_INCLUDE_DIR
    NAMES NvPipe.h
    PATHS include
)
find_library(NvPipe_LIBRARY
    NAMES NvPipe
    PATHS lib
)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(NvPipe
    DEFAULT_MSG
    NvPipe_INCLUDE_DIR NvPipe_LIBRARY
)

if(NvPipe_FOUND)
    set(NvPipe_LIBRARIES ${NvPipe_LIBRARY})
    set(NvPipe_INCLUDE_DIRS ${NvPipe_INCLUDE_DIR})
endif()

if(NvPipe_FOUND AND NOT TARGET NvPipe::NvPipe)
    add_library(NvPipe::NvPipe UNKNOWN IMPORTED)
    set_target_properties(NvPipe::NvPipe PROPERTIES
        IMPORTED_LOCATION "${NvPipe_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${NvPipe_INCLUDE_DIR}"
    )
endif()

mark_as_advanced(
    NvPipe_INCLUDE_DIR NvPipe_INCLUDE_DIRS
    NvPipe_LIBRARY NvPipe_LIBRARIES)
