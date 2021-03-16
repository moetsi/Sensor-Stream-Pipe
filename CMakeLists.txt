cmake_minimum_required(VERSION 3.11...3.19)
project(ssp)

set(CMAKE_CXX_STANDARD 11)

# Set default build type to Release if none is defined
set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Download dependencies
include(FetchContent)

if (APPLE)
  FetchContent_Declare(
    ssp_dep
    URL http://r2d3.geldreich.net/downloads/.20210308.ssp/202103161130__ssp_macdep.tar.gz
    URL_HASH MD5=8cdef0d14b1ef3be77f2a023fd399ee0
  )
elseif (WIN32)
  FetchContent_Declare(
    ssp_dep
    URL file://c:/Users/david/src/Sensor-Stream-Pipe/3rdparty/202103121929__ssp_macdep.tar.gz
    URL_HASH MD5=34d922b7c2af6da987ec2b275aaafb30
  )
endif()

FetchContent_GetProperties(ssp_dep)
if (NOT ssp_dep_POPULATED)
  message(STATUS "Populating SSP dependencies")
  FetchContent_Populate(ssp_dep)
endif()

if (ssp_dep_POPULATED)
  message(STATUS "Using SSP dependencies")
  set(cereal_DIR ${ssp_dep_SOURCE_DIR}/cereal/share/cmake/cereal CACHE PATH "cereal installation path")
  if (WIN32)
    set(OpenCV_STATIC ON)
    set(OpenCV_DIR ${ssp_dep_SOURCE_DIR}/opencv CACHE PATH "OpenCV installation path")
    set(ZeroMQ_DIR ${ssp_dep_SOURCE_DIR}/libzmq/CMake CACHE PATH "ZeroMQ installation path")
  else()
    set(OpenCV_DIR ${ssp_dep_SOURCE_DIR}/opencv/share/OpenCV CACHE PATH "OpenCV installation path")
    set(ZeroMQ_DIR ${ssp_dep_SOURCE_DIR}/libzmq/lib/cmake/ZeroMQ CACHE PATH "ZeroMQ installation path")
  endif()
  set(spdlog_DIR ${ssp_dep_SOURCE_DIR}/spdlog/lib/cmake/spdlog CACHE PATH "spdlog installation path")
  set(yaml-cpp_DIR ${ssp_dep_SOURCE_DIR}/yaml-cpp/lib/cmake/yaml-cpp CACHE PATH "yaml-cpp installation path")
  set(cppzmq_DIR ${ssp_dep_SOURCE_DIR}/cppzmq/share/cmake/cppzmq CACHE PATH "cppzmq installation path")
  set(FFMPEG_ROOT ${ssp_dep_SOURCE_DIR}/ffmpeg CACHE PATH "ffmpeg installation path")
  set(zdepth_DIR ${ssp_dep_SOURCE_DIR}/zdepth/lib/cmake/zdepth CACHE PATH "Zdepth installation path")
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Parallel build with VisualStudio
add_compile_options($<$<CXX_COMPILER_ID:MSVC>:/MP>)

find_package(Threads REQUIRED)
find_package(OpenCV COMPONENTS core imgproc highgui REQUIRED)
find_package(FFmpeg COMPONENTS AVCODEC AVUTIL AVFORMAT SWSCALE SWRESAMPLE REQUIRED)
find_package(cereal REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ZeroMQ REQUIRED)
find_package(cppzmq REQUIRED)
find_package(zdepth REQUIRED)

# TODO: modify FindFFmpeg to add this automatically
include_directories(${FFMPEG_INCLUDE_DIRS})

set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)

if (WIN32)
    option(SSP_WITH_KINECT_SUPPORT "Compile with Kinect support" ON)
    option(SSP_WITH_K4A_BODYTRACK "Compile with Body Tracking" ON)
    option(SSP_WITH_NVPIPE_SUPPORT "Compile with NvPipe" ON)
else()
    option(SSP_WITH_KINECT_SUPPORT "Compile with Kinect support" OFF)
    option(SSP_WITH_K4A_BODYTRACK "Compile with Body Tracking" OFF)
    option(SSP_WITH_NVPIPE_SUPPORT "Compile with NvPipe" OFF)
endif()
option(SSP_WITH_ZMQ_POLLING "Compile with ZeroMQ polling" OFF)

if (SSP_WITH_NVPIPE_SUPPORT)
    find_package(NvPipe)

    # If not found disable NvPipe support
    if (NvPipe_FOUND)
        add_definitions(-DSSP_WITH_NVPIPE_SUPPORT=1)
    else()
        set(SSP_WITH_NVPIPE_SUPPORT OFF)
        message(STATUS "Setting SSP_WITH_NVPIPE_SUPPORT to OFF")
    endif()
endif()

if (SSP_WITH_KINECT_SUPPORT)
    find_package(k4a)

    if (k4a_FOUND)
        add_definitions(-DSSP_WITH_KINECT_SUPPORT=1)
    else()
        set(SSP_WITH_KINECT_SUPPORT OFF)
        message(STATUS "Setting SSP_WITH_KINECT_SUPPORT to OFF")
    endif()
endif()

if (SSP_WITH_K4A_BODYTRACK)
    find_package(k4abt)

    if (k4abt_FOUND)
        add_definitions(-DSSP_WITH_K4A_BODYTRACK=1)
    else()
        set(SSP_WITH_K4A_BODYTRACK OFF)
        message(STATUS "Setting SSP_WITH_K4A_BODYTRACK to OFF")
    endif()
endif()

if (SSP_WITH_ZMQ_POLLING)
    add_definitions(-DSSP_WITH_ZMQ_POLLING=1)
endif ()

#########################
### Build the library
#########################

set(SOURCE_FILES
    readers/image_reader.cc
    readers/image_reader.h
    readers/multi_image_reader.cc
    readers/multi_image_reader.h
    structs/frame_struct.hpp
    utils/image_converter.cc
    utils/image_converter.h
    utils/image_decoder.cc
    utils/image_decoder.h
    utils/libav_types.h
    utils/logger.h
    utils/similarity_measures.cc
    utils/similarity_measures.h
    utils/utils.cc
    utils/utils.h
    utils/video_utils.cc
    utils/video_utils.h

    decoders/libav_decoder.cc
    decoders/libav_decoder.h
    decoders/zdepth_decoder.cc
    decoders/zdepth_decoder.h

    encoders/zdepth_encoder.cc
    encoders/zdepth_encoder.h
    encoders/null_encoder.cc
    encoders/null_encoder.h
    encoders/libav_encoder.cc
    encoders/libav_encoder.h

    readers/video_file_reader.cc
    readers/video_file_reader.h
    readers/image_reader.cc
    readers/image_reader.h
    readers/network_reader.cc
    readers/network_reader.h
)

if (SSP_WITH_NVPIPE_SUPPORT)
    list(APPEND SOURCE_FILES encoders/nv_encoder.cc decoders/nv_decoder.cc)
endif ()

if (SSP_WITH_KINECT_SUPPORT)
    list(APPEND SOURCE_FILES readers/kinect_reader.cc utils/kinect_utils.cc)
endif ()

add_library(ssp STATIC ${SOURCE_FILES})
target_link_libraries(ssp PRIVATE ${FFMPEG_LIBRARIES})

target_link_libraries(ssp PUBLIC
    cereal
    ${OpenCV_LIBS}
    spdlog::spdlog
    libzmq-static
    cppzmq-static
    ${YAML_CPP_LIBRARIES}
    zdepth::zdepth
)

if (SSP_WITH_NVPIPE_SUPPORT)
    target_link_libraries(ssp PUBLIC NvPipe::NvPipe)
endif ()

if (SSP_WITH_KINECT_SUPPORT)
    target_link_libraries(ssp PUBLIC k4a k4arecord depthengine)
endif ()

#############################
### Build the applications
#############################

add_library(ssp_plugin MODULE
    servers/ssp_server.cc
    clients/ssp_client_opencv.cc
    clients/ssp_client_template.cc
)
target_compile_definitions(ssp_plugin PRIVATE SSP_PLUGIN)
target_link_libraries(ssp_plugin ssp)

add_executable(ssp_client_opencv clients/ssp_client_opencv.cc)
target_link_libraries(ssp_client_opencv ssp)

add_executable(ssp_client_template clients/ssp_client_template.cc)
target_link_libraries(ssp_client_template ssp)

add_executable(ssp_server servers/ssp_server.cc)
target_link_libraries(ssp_server ssp)

add_executable(ssp_tester testers/ssp_tester.cc)
target_link_libraries(ssp_tester ssp)

if (SSP_WITH_K4A_BODYTRACK)
    add_executable(ssp_client_k4a clients/ssp_client_k4a.cc)
    target_link_libraries(ssp_client_k4a ssp k4abt)

    add_executable(ssp_client_pointcloud clients/ssp_client_pointcloud.cc)
    target_link_libraries(ssp_client_pointcloud ssp k4abt)
    
    target_sources(ssp_plugin PRIVATE clients/ssp_client_k4a.cc clients/ssp_client_pointcloud.cc)
    target_link_libraries(ssp_plugin ssp k4abt)
endif ()
