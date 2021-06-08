cmake_minimum_required(VERSION 3.11...3.19)
project(ssp)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

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

if (IOS)
  FetchContent_Declare(
    ssp_dep
    URL http://r2d3.geldreich.net/downloads/.20210308.ssp/202104091831__ssp_iosdep.tar.gz
    URL_HASH MD5=89578a5752c950f983b570b0b9a602b5
  )
elseif (APPLE)
  FetchContent_Declare(
    ssp_dep
    URL http://r2d3.geldreich.net/downloads/.20210308.ssp/202103161130__ssp_macdep.tar.gz
    URL_HASH MD5=8cdef0d14b1ef3be77f2a023fd399ee0
  )
elseif (WIN32)
  FetchContent_Declare(
    ssp_dep
    URL http://r2d3.geldreich.net/downloads/.20210308.ssp/202103181613__ssp_windep.tar.gz
    URL_HASH MD5=c228bcad89609f7fdec0b7b8f65d10bb
  )
elseif (UNIX)
  FetchContent_Declare(
    ssp_dep
    URL http://r2d3.geldreich.net/downloads/.20210308.ssp/202104061701__ssp_lindep.tar.gz
    URL_HASH MD5=4e6d2610a90874ec14a103eb57820626
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
    set(yaml-cpp_DIR ${ssp_dep_SOURCE_DIR}/yaml-cpp/CMake CACHE PATH "yaml-cpp installation path")
    set(k4a_DIR ${ssp_dep_SOURCE_DIR}/k4a/lib/cmake/k4a CACHE PATH "k4a installation path")
  else()
    set(OpenCV_DIR ${ssp_dep_SOURCE_DIR}/opencv/share/OpenCV CACHE PATH "OpenCV installation path")
    set(ZeroMQ_DIR ${ssp_dep_SOURCE_DIR}/libzmq/lib/cmake/ZeroMQ CACHE PATH "ZeroMQ installation path")
    set(yaml-cpp_DIR ${ssp_dep_SOURCE_DIR}/yaml-cpp/lib/cmake/yaml-cpp CACHE PATH "yaml-cpp installation path")
  endif()
  set(spdlog_DIR ${ssp_dep_SOURCE_DIR}/spdlog/lib/cmake/spdlog CACHE PATH "spdlog installation path")
  set(cppzmq_DIR ${ssp_dep_SOURCE_DIR}/cppzmq/share/cmake/cppzmq CACHE PATH "cppzmq installation path")
  set(FFMPEG_ROOT ${ssp_dep_SOURCE_DIR}/ffmpeg CACHE PATH "ffmpeg installation path")
  set(zdepth_DIR ${ssp_dep_SOURCE_DIR}/zdepth/lib/cmake/zdepth CACHE PATH "Zdepth installation path")
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Parallel build with VisualStudio
add_compile_options($<$<CXX_COMPILER_ID:MSVC>:/MP>)

find_package(Threads REQUIRED)
find_package(cereal REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ZeroMQ REQUIRED)
find_package(cppzmq REQUIRED)
find_package(zdepth REQUIRED)

if (IOS)
  add_definitions(-DFFMPEG_AS_FRAMEWORK)
  set(FFMPEG_LIBRARIES
    ${ssp_dep_SOURCE_DIR}/lib/FFmpeg.framework
    -liconv -lm -lbz2 -lz -pthread
    "-framework AudioToolbox"
    "-framework CoreFoundation"
    "-framework CoreMedia"
    "-framework CoreServices"
    "-framework CoreVideo"
    "-framework Security"
    "-framework VideoToolbox"
  )
  set(OpenCV_LIBS ${ssp_dep_SOURCE_DIR}/lib/opencv2.framework)
else()
  find_package(FFmpeg COMPONENTS AVCODEC AVUTIL AVFORMAT SWSCALE SWRESAMPLE REQUIRED)
  find_package(OpenCV COMPONENTS core imgproc highgui REQUIRED)
endif()

set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# TODO: modify FindFFmpeg to add this automatically
include_directories(${FFMPEG_INCLUDE_DIRS})

if (WIN32)
    file(COPY ${FFMPEG_INCLUDE_DIRS}/../bin/
         DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Release
         FILES_MATCHING PATTERN "*.dll")
endif ()

if (APPLE)
    option(SSP_WITH_KINECT_SUPPORT "Compile with Kinect support" OFF)
    option(SSP_WITH_K4A_BODYTRACK "Compile with Body Tracking" OFF)
    option(SSP_WITH_NVPIPE_SUPPORT "Compile with NvPipe" OFF)
else()
    option(SSP_WITH_KINECT_SUPPORT "Compile with Kinect support" ON)
    option(SSP_WITH_K4A_BODYTRACK "Compile with Body Tracking" ON)
    option(SSP_WITH_NVPIPE_SUPPORT "Compile with NvPipe" ON)
endif()
option(SSP_WITH_ZMQ_POLLING "Compile with ZeroMQ polling" OFF)

if (SSP_WITH_NVPIPE_SUPPORT)
    find_package(NvPipe CONFIG)

    # If not found disable NvPipe support
    if (NvPipe_FOUND)
        add_definitions(-DSSP_WITH_NVPIPE_SUPPORT=1)
    else()
        set(SSP_WITH_NVPIPE_SUPPORT OFF)
        message(STATUS "Setting SSP_WITH_NVPIPE_SUPPORT to OFF")
    endif()
endif()

if (SSP_WITH_KINECT_SUPPORT)
    find_package(k4a CONFIG)

    if (k4a_FOUND)
        add_definitions(-DSSP_WITH_KINECT_SUPPORT=1)

        if (WIN32)
            file(COPY ${k4a_DIR}/../../../bin/
                 DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Release
                 FILES_MATCHING PATTERN "*.dll")
        endif ()
    else()
        set(SSP_WITH_KINECT_SUPPORT OFF)
        message(STATUS "Setting SSP_WITH_KINECT_SUPPORT to OFF")
    endif()
endif()

if (SSP_WITH_K4A_BODYTRACK)
    find_package(k4abt)

    if (k4abt_FOUND)
        add_definitions(-DSSP_WITH_K4A_BODYTRACK=1)

        if (WIN32)
            message(STATUS "Found Kinect Body Tracking: ${k4abt_ROOT}")
            file(COPY ${k4abt_ROOT}/windows-desktop/amd64/release/bin/
                 DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Release
                 FILES_MATCHING PATTERN "*.dll")
        endif ()
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
    list(APPEND SOURCE_FILES encoders/nv_encoder.cc encoders/nv_encoder.h decoders/nv_decoder.cc decoders/nv_decoder.h)
endif ()

if (SSP_WITH_KINECT_SUPPORT)
    list(APPEND SOURCE_FILES readers/kinect_reader.cc readers/kinect_reader.h utils/kinect_utils.cc utils/kinect_utils.h)
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

if (IOS)
    target_sources(ssp PRIVATE readers/iphone_reader.mm readers/iphone_reader.h)
    target_link_libraries(ssp PRIVATE "-framework ARKit")
endif ()

if (SSP_WITH_NVPIPE_SUPPORT)
    target_link_libraries(ssp PUBLIC NvPipe)
endif ()

if (SSP_WITH_KINECT_SUPPORT)
    target_link_libraries(ssp PUBLIC k4a::k4a)
endif ()

if (SSP_WITH_K4A_BODYTRACK)
    target_link_libraries(ssp PUBLIC k4a::k4abt)
endif ()


#############################
### Build the applications
#############################

if (IOS)
  set(PLUGIN_TYPE STATIC)
else()
  set(PLUGIN_TYPE MODULE)
endif()

add_library(ssp_plugin ${PLUGIN_TYPE}
    servers/ssp_server.cc
    clients/ssp_client_opencv.cc
    clients/ssp_client_template.cc
)
target_compile_definitions(ssp_plugin PRIVATE SSP_PLUGIN)
target_link_libraries(ssp_plugin ssp)

if (IOS)
  # Pack all the needed libraries for Unity3D iOS usage
  add_custom_target(unity
          ${CMAKE_SOURCE_DIR}/cmake/package_for_unity.sh
          DEPENDS ssp_plugin)
endif()

add_executable(ssp_client_opencv clients/ssp_client_opencv.cc)
target_link_libraries(ssp_client_opencv ssp)

add_executable(ssp_client_template clients/ssp_client_template.cc)
target_link_libraries(ssp_client_template ssp)

add_executable(ssp_server servers/ssp_server.cc)
target_link_libraries(ssp_server ssp)

add_executable(ssp_tester testers/ssp_tester.cc)
target_link_libraries(ssp_tester ssp)

if (IOS)
    set_source_files_properties(servers/ssp_server.cc PROPERTIES
                            COMPILE_FLAGS "-x objective-c++")
    cmake_host_system_information(RESULT HOSTNAME QUERY HOSTNAME)
    configure_file(configs/serve_ios_raw.yaml.in serve_ios_raw.yaml)
    set(IOS_RESOURCES serve_ios_raw.yaml)
    target_sources(ssp_server PUBLIC ${IOS_RESOURCES})
    target_link_libraries(ssp_server "-framework UIKit")
    set_source_files_properties(${IOS_RESOURCES}
      PROPERTIES MACOSX_PACKAGE_LOCATION Resources)

    set_target_properties(
      ssp_client_opencv
      ssp_client_template
      ssp_server
      ssp_tester
      PROPERTIES
      XCODE_ATTRIBUTE_DEVELOPMENT_TEAM "4A95ACP6M5"
      XCODE_ATTRIBUTE_PRODUCT_BUNDLE_IDENTIFIER "com.moetsi.\${PRODUCT_NAME:identifier}"
      MACOSX_BUNDLE_BUNDLE_VERSION "1.0"
      MACOSX_BUNDLE_SHORT_VERSION_STRING "1.0"
      MACOSX_BUNDLE_GUI_IDENTIFIER "com.moetsi.\${PRODUCT_NAME:identifier}"
      MACOSX_BUNDLE_INFO_PLIST ${CMAKE_SOURCE_DIR}/servers/Info.plist.in
    )
endif()

if (SSP_WITH_K4A_BODYTRACK)
    add_executable(ssp_client_k4a clients/ssp_client_k4a.cc)
    target_link_libraries(ssp_client_k4a ssp)

    add_executable(ssp_client_pointcloud clients/ssp_client_pointcloud.cc)
    target_link_libraries(ssp_client_pointcloud ssp)

    target_sources(ssp_plugin PRIVATE clients/ssp_client_k4a.cc)
endif ()
