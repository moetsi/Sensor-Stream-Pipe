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
set(SOURCE_FILES readers/ImageReader.cpp structs/FrameStruct.hpp utils/Utils.cpp readers/IReader.h)

add_executable(ssp_client clients/SSPClient.cpp readers/NetworkReader.cpp utils/VideoUtils.cpp decoders/LibAvDecoder.cpp decoders/NvDecoder.cpp utils/ImageDecoder.cpp decoders/ZDepthDecoder.cpp ${SOURCE_FILES})
add_executable(ssp_server servers/SSPServer.cpp readers/VideoFileReader.cpp readers/KinectReader.cpp encoders/LibAvEncoder.cpp utils/VideoUtils.cpp utils/ImageDecoder.cpp utils/KinectUtils.cpp utils/SimilarityMeasures.cpp encoders/ZDepthEncoder.cpp encoders/NvEncoder.cpp encoders/NullEncoder.cpp decoders/LibAvDecoder.cpp decoders/NvDecoder.cpp decoders/ZDepthDecoder.cpp ${SOURCE_FILES} utils/Logger.h)
add_executable(ssp_tester testers/SSPTester.cpp utils/VideoUtils.cpp decoders/LibAvDecoder.cpp decoders/NvDecoder.cpp readers/VideoFileReader.cpp readers/KinectReader.cpp encoders/LibAvEncoder.cpp utils/VideoUtils.cpp utils/ImageDecoder.cpp encoders/ZDepthEncoder.cpp decoders/ZDepthDecoder.cpp utils/KinectUtils.cpp utils/SimilarityMeasures.cpp encoders/NvEncoder.cpp encoders/NullEncoder.cpp decoders/LibAvDecoder.cpp decoders/NvDecoder.cpp ${SOURCE_FILES})

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
