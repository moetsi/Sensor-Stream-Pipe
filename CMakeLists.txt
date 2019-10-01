cmake_minimum_required(VERSION 3.10.2)
project(davp-baseline)

include_directories("$ENV{HOME}/libs/include")
link_directories("$ENV{HOME}/libs/lib")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules" "$ENV{HOME}/libs/lib")
list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/libs/lib")

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)
find_package(FFmpeg REQUIRED)

if (FFMPEG_FOUND)
    #  FFMPEG_INCLUDE_DIRS  - Include directory necessary for using the required components headers.
    #  FFMPEG_LIBRARIES     - Link these to use the required ffmpeg components.
    #  FFMPEG_DEFINITIONS   - Compiler switches required for using the required ffmpeg components.
    message("FFMPEG_INCLUDE_DIRS = ${FFMPEG_INCLUDE_DIRS} ")
    message("FFMPEG_LIBRARIES = ${FFMPEG_LIBRARIES} ")
    message("FFMPEG_DEFINITIONS = ${FFMPEG_DEFINITIONS} ")
endif ()

set(EXECUTABLE_OUTPUT_PATH bin)
set(SOURCE_FILES readers/image_reader.cc structs/frame_struct.hpp utils/utils.cc readers/ireader.h)

add_executable(ssp_client clients/ssp_client_opencv.cc readers/network_reader.cc utils/video_utils.cc decoders/libav_decoder.cc decoders/nv_decoder.cc utils/image_decoder.cc decoders/zdepth_decoder.cc ${SOURCE_FILES})
add_executable(ssp_server servers/ssp_server.cc readers/video_file_reader.cc readers/kinect_reader.cc encoders/libav_encoder.cc utils/video_utils.cc utils/image_decoder.cc utils/kinect_utils.cc utils/similarity_measures.cc encoders/zdepth_encoder.cc encoders/nv_encoder.cc encoders/null_encoder.cc decoders/libav_decoder.cc decoders/nv_decoder.cc decoders/zdepth_decoder.cc ${SOURCE_FILES} utils/logger.h)
add_executable(ssp_tester testers/ssp_tester.cc utils/video_utils.cc decoders/libav_decoder.cc decoders/nv_decoder.cc readers/video_file_reader.cc readers/kinect_reader.cc encoders/libav_encoder.cc utils/video_utils.cc utils/image_decoder.cc encoders/zdepth_encoder.cc decoders/zdepth_decoder.cc utils/kinect_utils.cc utils/similarity_measures.cc encoders/nv_encoder.cc encoders/null_encoder.cc decoders/libav_decoder.cc decoders/nv_decoder.cc ${SOURCE_FILES})

target_link_libraries(ssp_client swscale)
target_link_libraries(ssp_client zmq)
target_link_libraries(ssp_client ${FFMPEG_LIBRARIES})
target_link_libraries(ssp_client ${OpenCV_LIBS})
target_link_libraries(ssp_client NvPipe)
target_link_libraries(ssp_client zdepth)
target_link_libraries(ssp_client yaml-cpp)
target_link_libraries(ssp_client zstd)
target_link_libraries(ssp_client k4a)

target_link_libraries(ssp_server swscale)
target_link_libraries(ssp_server zmq)
target_link_libraries(ssp_server ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(ssp_server ${FFMPEG_LIBRARIES})
target_link_libraries(ssp_server ${OpenCV_LIBS})
target_link_libraries(ssp_server k4a)
target_link_libraries(ssp_server depthengine)
target_link_libraries(ssp_server k4arecord)
target_link_libraries(ssp_server yaml-cpp)
target_link_libraries(ssp_server NvPipe)
target_link_libraries(ssp_server zdepth)
target_link_libraries(ssp_server zstd)

target_link_libraries(ssp_tester swscale)
target_link_libraries(ssp_tester ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(ssp_tester ${FFMPEG_LIBRARIES})
target_link_libraries(ssp_tester ${OpenCV_LIBS})
target_link_libraries(ssp_tester k4a)
target_link_libraries(ssp_tester depthengine)
target_link_libraries(ssp_tester k4arecord)
target_link_libraries(ssp_tester yaml-cpp)
target_link_libraries(ssp_tester NvPipe)
target_link_libraries(ssp_tester zdepth)
target_link_libraries(ssp_tester zstd)
