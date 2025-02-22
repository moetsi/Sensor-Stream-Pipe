cmake_minimum_required(VERSION 3.11...3.19)
project(ssp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=1)
# add_compile_options("-Wall" "-Wextra" "-Wshadow" "-Wno-unused-result" "-Wabi-tag")

option(LIBOPENH264_ENABLED "Enable libh264 codec" ON)

# Check if looking for oakd+openvino dependencies (for oakd_xlink_reader.cc)
SET(XLINK_ENABLED FALSE CACHE BOOL "xlink enabled")
message(STATUS "XLINK_ENABLED ${XLINK_ENABLED}")

# Check if looking for k4a dependencies (for kinect_reader.cc)
SET(K4A_ENABLED FALSE CACHE BOOL "kinect enabled")
message(STATUS "K4A_ENABLED ${K4A_ENABLED}")

# Set default build type to Release if none is defined
set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

option(LOCAL_DEP_BUILD_LINUX "Build deps locally on linux with some options" OFF)
if (LOCAL_DEP_BUILD_LINUX)
# add script here?
endif()
option(LOCAL_DEP_BUILD_WINDOWS "Build deps locally on windows with some options" OFF)
if (LOCAL_DEP_BUILD_WINDOWS)
# add script here?
endif()

# Download dependencies
include(FetchContent)

if (IOS)
  FetchContent_Declare(
    ssp_dep
    URL https://moetsiblob.blob.core.windows.net/binaries-ssp/202104091831__ssp_iosdep.tar.gz
    URL_HASH MD5=89578a5752c950f983b570b0b9a602b5
  )
elseif (APPLE)
  if (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    message(STATUS "Detected Apple Intel CPU")
    FetchContent_Declare(
      ssp_dep
      URL https://moetsiblob.blob.core.windows.net/binaries-ssp/202103161130__ssp_macdep_intel.tar.gz
      URL_HASH MD5=8cdef0d14b1ef3be77f2a023fd399ee0
    )
  elseif (CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
    message(STATUS "Detected Apple Silicon CPU")
    execute_process(
      COMMAND ${CMAKE_SOURCE_DIR}/3rdparty/build_mac_dep.sh
      RESULT_VARIABLE script_result
    )
    if(script_result AND NOT script_result STREQUAL "0")
      message(FATAL_ERROR "Failed to run build_mac_dep.sh")
    endif()
    FetchContent_Declare(
      ssp_dep
      SOURCE_DIR ${CMAKE_SOURCE_DIR}/3rdparty/tmp/local.ssp
    )
  endif()
elseif (WIN32)
  if (LOCAL_DEP_BUILD_WINDOWS)
  message(STATUS "please make sure you have gone to 3rdpary and run build_win_dep.sh")
  if (WIN32)
    message("Windows use this and download 2022.3: https://docs.openvino.ai/2023.0/openvino_docs_install_guides_installing_openvino_from_archive_windows.html")
    message("You will need to run C:/Program Files (x86)/Intel/openvino_2022/setupvars.bat")
    message("You will get an error like: || By not providing Findngraph.cmake in CMAKE_MODULE_PATH this project || if you do not have OpenVINO installed nor run setupvars.sh")
  endif ()
  FetchContent_Declare(
    ssp_dep
    SOURCE_DIR ${CMAKE_SOURCE_DIR}/3rdparty/tmp/local.ssp
  )
  else ()
    if (XLINK_ENABLED)
      if (WIN32)
        message("Windows use this and download 2022.3: https://docs.openvino.ai/2023.0/openvino_docs_install_guides_installing_openvino_from_archive_windows.html")
        message("You will need to run C:/Program Files (x86)/Intel/openvino_2022/setupvars.bat")
        message("You will get an error like: || By not providing Findngraph.cmake in CMAKE_MODULE_PATH this project || if you do not have OpenVINO installed nor run setupvars.sh")
      endif ()
        FetchContent_Declare(
        ssp_dep
        # URL https://moetsiblob.blob.core.windows.net/binaries-ssp/202110161452_ssp_windep_xlink_enabled.tar.gz
        # URL_HASH MD5=fe027d0a694c2c0447d8f29ea2812362

        # new with release config
        URL https://moetsiblob.blob.core.windows.net/binaries-ssp/202110181158_ssp_windep_xlink_enabled.tar.gz
        URL_HASH MD5=f57eb06fd32d7ab47221854dbda0365e
        
        # rebuilding using Visual Studio 16 2019
        # URL https://moetsiblob.blob.core.windows.net/binaries-ssp/202110162220_ssp_windep_xlink_enabled.tar.gz
        # URL_HASH MD5=dc804195928ac3f36d93d28664d024f1
    )
    else ()
      FetchContent_Declare(
        ssp_dep
        # old
        URL https://moetsiblob.blob.core.windows.net/binaries-ssp/202103181613__ssp_windep.tar.gz
        URL_HASH MD5=c228bcad89609f7fdec0b7b8f65d10bb

        # # new checking if works
        # URL https://moetsiblob.blob.core.windows.net/binaries-ssp/202110161442_ssp_windep.tar.gz
        # URL_HASH MD5=da1888c4d931ac15750afeabac6eb6ce
    )
    endif()
  endif()
elseif (UNIX)
  if (LOCAL_DEP_BUILD_LINUX)
    message(STATUS "please make sure you have gone to 3rdpary and run build_lin_dep.sh")
    FetchContent_Declare(
      ssp_dep
      SOURCE_DIR ${CMAKE_SOURCE_DIR}/3rdparty/tmp/local.ssp
    )
  else ()
    if (XLINK_ENABLED)
      message("Remember to compile with OpenVINO you will need to run [setupvars.sh] in opt/intel/openvino_2021/bin (or where ever you installed openvino and that script landed) to get OpenVINO environment initialized. See OpenVINO documentation if confused about this setupvars.sh message.")
      message("You will get an error like: || By not providing Findngraph.cmake in CMAKE_MODULE_PATH this project || if you do not have OpenVINO installed nor run setupvars.sh")
      FetchContent_Declare(
        ssp_dep
        URL https://moetsiblob.blob.core.windows.net/binaries-ssp/202201211927__ssp_lindep.tar.gz
        URL_HASH MD5=3b931ec54c5d377b8ef4466903339bb6
      )
    else ()
      FetchContent_Declare(
        ssp_dep
        URL https://moetsiblob.blob.core.windows.net/binaries-ssp/202104061701__ssp_lindep.tar.gz
        URL_HASH MD5=4e6d2610a90874ec14a103eb57820626
      )
    endif()
  endif()
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
    set(OpenCV_DIR ${ssp_dep_SOURCE_DIR}/opencv/lib/cmake/opencv4 CACHE PATH "OpenCV installation path")
    set(ZeroMQ_DIR ${ssp_dep_SOURCE_DIR}/libzmq/lib/cmake/ZeroMQ CACHE PATH "ZeroMQ installation path")
    set(yaml-cpp_DIR ${ssp_dep_SOURCE_DIR}/yaml-cpp/lib/cmake/yaml-cpp CACHE PATH "yaml-cpp installation path")
    set(depthai_DIR ${ssp_dep_SOURCE_DIR}/depthai-core/lib/cmake/depthai CACHE PATH "depthai installation path")
  endif()
  set(spdlog_DIR ${ssp_dep_SOURCE_DIR}/spdlog/lib/cmake/spdlog CACHE PATH "spdlog installation path")
  set(cppzmq_DIR ${ssp_dep_SOURCE_DIR}/cppzmq/share/cmake/cppzmq CACHE PATH "cppzmq installation path")
  set(FFMPEG_ROOT ${ssp_dep_SOURCE_DIR}/ffmpeg CACHE PATH "ffmpeg installation path")
  set(zdepth_DIR ${ssp_dep_SOURCE_DIR}/zdepth/lib/cmake/zdepth CACHE PATH "Zdepth installation path")
  if (LOCAL_DEP_BUILD_LINUX)
    if (LIBOPENH264_ENABLED)
      message(STATUS " searching for openh264 in " ${ssp_dep_SOURCE_DIR}/openh264/)
      set(openh264_INCLUDE_DIR ${ssp_dep_SOURCE_DIR}/openh264/include)
      set(openh264_DIR ${ssp_dep_SOURCE_DIR}/openh264)
      set(openh264_LIBRARY ${ssp_dep_SOURCE_DIR}/openh264/lib/libopenh264.so.6) # yes this is linux for now
    endif()
  endif()
endif()

if (LIBOPENH264_ENABLED)
add_definitions(-DLIBOPENH264_ENABLED)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Parallel build with VisualStudio
add_compile_options($<$<CXX_COMPILER_ID:MSVC>:/MP>)

option(ASAN "Build with ASAN" OFF)
if (ASAN)
  add_compile_options("-fsanitize=address")
  add_link_options("-fsanitize=address")
endif(ASAN)

option(TSAN "Build with TSAN" OFF)
if (TSAN)
  add_compile_options("-fsanitize=thread")
  add_link_options("-fsanitize=thread")
endif(TSAN)

find_package(Threads REQUIRED)
find_package(cereal REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ZeroMQ REQUIRED)
find_package(cppzmq REQUIRED)
find_package(zdepth REQUIRED)

if (LOCAL_DEP_BUILD_LINUX)
  if (LIBOPENH264_ENABLED)
    find_package(openh264 REQUIRED)
  endif()
endif()

option(SSP_WITH_DEPTHAI_SUPPORT "Compile with DepthAI support" OFF)

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
  if (SSP_WITH_DEPTHAI_SUPPORT)
    find_package(depthai CONFIG REQUIRED)
  endif()
  find_package(OpenCV COMPONENTS core imgproc highgui REQUIRED PATHS ${ssp_dep_SOURCE_DIR}/opencv/share/OpenCV NO_DEFAULT_PATH)
endif()

if(OpenCV_FOUND)
  message(STATUS "Found OpenCV")
  message(STATUS "Includes: " ${OpenCV_INCLUDE_DIRS})
else(OpenCV_FOUND)
  message(STATUS "Cannot find OpenCV")
endif(OpenCV_FOUND)

set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# TODO: modify FindFFmpeg to add this automatically
include_directories(${FFMPEG_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV_DIR = " ${OpenCV_DIR})

if (WIN32)
    file(COPY ${FFMPEG_INCLUDE_DIRS}/../bin/
         DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Release
         FILES_MATCHING PATTERN "*.dll")
endif ()

if (APPLE)
    option(SSP_WITH_KINECT_SUPPORT "Compile with Kinect support" OFF)
    option(SSP_WITH_K4A_BODYTRACK "Compile with Body Tracking" OFF)
    SET(K4A_ENABLED FALSE)
    option(SSP_WITH_NVPIPE_SUPPORT "Compile with NvPipe" OFF)
else()

    option(SSP_WITH_KINECT_SUPPORT "Compile with Kinect support" OFF)
    option(SSP_WITH_K4A_BODYTRACK "Compile with Body Tracking" OFF)
    if (NOT K4A_ENABLED)
      set(SSP_WITH_KINECT_SUPPORT OFF)
      set(SSP_WITH_K4A_BODYTRACK OFF)
    endif()
    option(SSP_WITH_NVPIPE_SUPPORT "Compile with NvPipe" OFF)
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
            include_directories(${k4abt_ROOT}/include)
        endif ()
    else()
        set(SSP_WITH_K4A_BODYTRACK OFF)
        message(STATUS "Setting SSP_WITH_K4A_BODYTRACK to OFF")
    endif()
endif()

if (SSP_WITH_ZMQ_POLLING)
    add_definitions(-DSSP_WITH_ZMQ_POLLING=1)
endif ()

if (XLINK_ENABLED)
    set(depthai_DIR ${ssp_dep_SOURCE_DIR}/depthai-core/lib/cmake/depthai CACHE PATH "DepthAI installation path")
    find_package(depthai CONFIG)
    # If not found disable DepthAI support
    if (depthai_FOUND)
        message(STATUS "Setting SSP_WITH_DEPTHAI_SUPPORT to ON")
        # This is the new OpenVINO API approach that we have not converted to

        # We have to define where to look for the OPENVINO package on windows (C:/Intel/openvino_NAMEOFFOLDER)
        if (WIN32)
          set(OpenVINO_DIR "C:/Program Files (x86)/Intel/openvino_2022/runtime/cmake")
        endif()
        find_package(OpenVINO REQUIRED)
        message(STATUS "Found OpenVINO: ${OpenVINO_DIR}")
        # This is the old OpenVINO API approach
        # find_package(ngraph REQUIRED)
        # find_package(InferenceEngine CONFIG REQUIRED)
        # include_directories(${InferenceEngine_INCLUDE_DIRS})
        # message(STATUS "Found InferenceEngine: ${InferenceEngine_INCLUDE_DIRS}")
        add_definitions(-DSSP_WITH_DEPTHAI_SUPPORT=1)
        include_directories(${ssp_dep_SOURCE_DIR}/depthai-core/include)
        include_directories(${ssp_dep_SOURCE_DIR}/depthai-core/include/depthai-shared/3rdparty)
        include_directories(${ssp_dep_SOURCE_DIR}/depthai-core/lib/cmake/depthai/dependencies/include)
        add_definitions(-D'DEPTHAI_TARGET_OPENCV')
        add_definitions(-D'DEPTHAI_TARGET_CORE')
        add_definitions(-D'JSON_USE_IMPLICIT_CONVERSIONS=1')
        add_definitions(-D'__PC__')
        add_definitions(-D'XLINK_USE_MX_ID_NAME=ON')
    else()
        set(SSP_WITH_DEPTHAI_SUPPORT OFF)
        message(STATUS "Setting SSP_WITH_DEPTHAI_SUPPORT to OFF")
    endif()
endif()

#########################
### Build the library
#########################

set(SOURCE_FILES
    readers/image_reader.cc
    readers/image_reader.h
    readers/multi_image_reader.cc
    readers/multi_image_reader.h
    structs/frame_struct.h
    structs/body_struct.h
    structs/detection_struct.h

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

    decoders/idecoder.cc
    decoders/libav_decoder.cc
    decoders/libav_decoder.h
    decoders/zdepth_decoder.cc
    decoders/zdepth_decoder.h

    encoders/iencoder.cc
    encoders/zdepth_encoder.cc
    encoders/zdepth_encoder.h
    encoders/null_encoder.cc
    encoders/null_encoder.h
    encoders/libav_encoder.cc
    encoders/libav_encoder.h

    readers/ireader.cc
    readers/video_file_reader.cc
    readers/video_file_reader.h
    readers/image_reader.cc
    readers/image_reader.h
    readers/dummy_body_reader.cc
    readers/dummy_body_reader.h
    readers/network_reader.cc
    readers/network_reader.h
)

# Checking for XLINK_ENABLED and if exists adding readers
if (UNIX)
  if (XLINK_ENABLED)
    list(APPEND SOURCE_FILES
      readers/oakd_xlink_full_reader.cc
      readers/oakd_xlink_full_reader.h
      readers/human_poses.cpp
      readers/human_poses.h
      
    )
    file(GLOB_RECURSE TRACKER_FILES tracker/*.cc tracker/*.cpp tracker/*.h tracker/*.hpp)
    list(APPEND SOURCE_FILES ${TRACKER_FILES})
    # Search for tracker header files in these folders
    include_directories(tracker/include)
    include_directories(tracker/utils/include)
  endif ()
endif ()

if (WIN32)
  if (XLINK_ENABLED)
    list(APPEND SOURCE_FILES
      readers/oakd_xlink_full_reader.cc
      readers/oakd_xlink_full_reader.h
      readers/human_poses.cpp
      readers/human_poses.h
      
    )
    file(GLOB_RECURSE TRACKER_FILES tracker/*.cc tracker/*.cpp tracker/*.h tracker/*.hpp)
    list(APPEND SOURCE_FILES ${TRACKER_FILES})
    # Search for tracker header files in these folders
    include_directories(tracker/include)
    include_directories(tracker/utils/include)
    # We have to use exact includes in windows
    include_directories("C:/Program Files (x86)/Intel/openvino_2022/runtime/include/ie")
    include_directories("C:/Program Files (x86)/Intel/openvino_2022/runtime/include")
    # We need to copy the dll in windows
    file(GLOB OPENVINO_DLLS
         "C:/Program Files (x86)/Intel/openvino_2022/runtime/bin/intel64/Release/*.dll")
    file(COPY ${OPENVINO_DLLS}
         DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Release)
    add_compile_options("/Zc:preprocessor") 
  endif ()
endif ()
# Done with adjusting for XLINK_ENABLED

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

if (openh264_FOUND)
    message(STATUS "adding openh264 to link libraries")
    target_link_libraries(ssp PUBLIC openh264::openh264)
endif()

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

if (XLINK_ENABLED)
  target_link_libraries(ssp PRIVATE depthai::opencv)
  target_link_libraries(ssp PRIVATE ${InferenceEngine_LIBRARIES} ${NGRAPH_LIBRARIES})
  target_link_libraries(ssp PRIVATE openvino::runtime)
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

add_library(raas_plugin SHARED
    clients/ssp_client_raas.cc
)
target_link_libraries(raas_plugin ssp)
add_custom_command(
  TARGET raas_plugin
  POST_BUILD
  COMMAND ${CMAKE_SOURCE_DIR}/cmake/raas_package_for_unity.sh "${CMAKE_BINARY_DIR}" "${CMAKE_SOURCE_DIR}"
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  COMMENT "Packaging libraries for Unity3D RaaS usage"
)

if (IOS)
  # Pack all the needed libraries for Unity3D iOS usage
  # Usage:
  # cmake --build . --config Release --target unity
  # mv ssp_plugin_unity ~/UnityProject/Assets/Plugins/iOS
  add_custom_target(unity
          ${CMAKE_SOURCE_DIR}/cmake/package_for_unity.sh
          DEPENDS ssp_plugin)
endif()

add_executable(ssp_client_opencv clients/ssp_client_opencv.cc)
target_link_libraries(ssp_client_opencv ssp)

add_executable(ssp_client_raas_playground clients/ssp_client_raas_playground.cc)
target_link_libraries(ssp_client_raas_playground ssp)

add_executable(ssp_client_body_logger clients/ssp_client_body_logger.cc)
target_link_libraries(ssp_client_body_logger ssp)

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

if (LOCAL_DEP_BUILD_LINUX)
  message(STATUS "in order to be able to run the server, some Linux machines require to export an LD_LIBRARY_PATH"
          " that points to libswresample.so.3 and libopenh264.so.6"
          " ex: export LD_LIBRARY_PATH=$ProjectDir/3rdparty/tmp/local.ssp/ffmpeg/lib:$ProjectDir/3rdparty/tmp/local.ssp/openh264/lib; ./ssp_server serve_video.yaml "
          " ... "
  )
endif()
