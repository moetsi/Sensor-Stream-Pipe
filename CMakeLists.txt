cmake_minimum_required(VERSION 3.10.2)
project(davp-baseline)

include_directories("$ENV{HOME}/libs/include")
link_directories("$ENV{HOME}/libs/lib")

find_package(Threads)
find_package(OpenCV REQUIRED)
find_package(Yaml-cpp)


set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)
find_package(FFmpeg REQUIRED) #add here the list of ffmpeg components required

if (FFMPEG_FOUND)
    #  FFMPEG_INCLUDE_DIRS  - Include directory necessary for using the required components headers.
    #  FFMPEG_LIBRARIES     - Link these to use the required ffmpeg components.
    #  FFMPEG_DEFINITIONS   - Compiler switches required for using the required ffmpeg components.
    message("FFMPEG_INCLUDE_DIRS = ${FFMPEG_INCLUDE_DIRS} ")
    message("FFMPEG_LIBRARIES = ${FFMPEG_LIBRARIES} ")
    message("FFMPEG_DEFINITIONS = ${FFMPEG_DEFINITIONS} ")
endif ()

set(EXECUTABLE_OUTPUT_PATH bin)
set(SOURCE_FILES readers/FrameReader.cpp structs/FrameStruct.hpp utils/Utils.cpp readers/IReader.h)

add_executable(client clients/FrameProcessingClient.cpp utils/VideoUtils.cpp ${SOURCE_FILES})
add_executable(frame_server servers/FrameServer.cpp ${SOURCE_FILES})
add_executable(video_file_server servers/VideoFileServer.cpp readers/VideoFileReader.cpp utils/VideoUtils.cpp ${SOURCE_FILES})
add_executable(kinect_server servers/KinectServer.cpp readers/KinectReader.cpp utils/VideoUtils.cpp utils/KinectUtils.cpp ${SOURCE_FILES})
add_executable(kinect_encoder_server servers/KinectEncoderServer.cpp readers/KinectReader.cpp encoders/FrameEncoder.cpp utils/VideoUtils.cpp utils/ImageDecoder.cpp utils/KinectUtils.cpp ${SOURCE_FILES})
add_executable(video_encoder_server servers/VideoEncoderServer.cpp encoders/FrameEncoder.cpp utils/VideoUtils.cpp utils/ImageDecoder.cpp ${SOURCE_FILES})
add_executable(video_encoder_test utils/VideoEncodingTester.cpp encoders/FrameEncoder.cpp utils/VideoEncodingTester.cpp utils/VideoUtils.cpp utils/ImageDecoder.cpp utils/SimilarityMeasures.cpp ${SOURCE_FILES})

target_link_libraries(client swscale)
target_link_libraries(client zmq)
target_link_libraries(client ${FFMPEG_LIBRARIES})
target_link_libraries(client ${OpenCV_LIBS})

target_link_libraries(frame_server zmq)
target_link_libraries(frame_server ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(frame_server ${OpenCV_LIBS})

target_link_libraries(kinect_server swscale)
target_link_libraries(kinect_server zmq)
target_link_libraries(kinect_server ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(kinect_server ${FFMPEG_LIBRARIES})
target_link_libraries(kinect_server ${OpenCV_LIBS})
target_link_libraries(kinect_server k4a)
target_link_libraries(kinect_server depthengine)
target_link_libraries(kinect_server k4arecord)
target_link_libraries(kinect_server yaml-cpp)

target_link_libraries(kinect_encoder_server swscale)
target_link_libraries(kinect_encoder_server zmq)
target_link_libraries(kinect_encoder_server ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(kinect_encoder_server ${FFMPEG_LIBRARIES})
target_link_libraries(kinect_encoder_server ${OpenCV_LIBS})
target_link_libraries(kinect_encoder_server k4a)
target_link_libraries(kinect_encoder_server depthengine)
target_link_libraries(kinect_encoder_server k4arecord)
target_link_libraries(kinect_encoder_server yaml-cpp)


target_link_libraries(video_file_server swscale)
target_link_libraries(video_file_server zmq)
target_link_libraries(video_file_server ${FFMPEG_LIBRARIES})
target_link_libraries(video_file_server ${OpenCV_LIBS})

target_link_libraries(video_encoder_server swscale)
target_link_libraries(video_encoder_server zmq)
target_link_libraries(video_encoder_server ${FFMPEG_LIBRARIES})
target_link_libraries(video_encoder_server ${OpenCV_LIBS})
target_link_libraries(video_encoder_server yaml-cpp)

target_link_libraries(video_encoder_test ${FFMPEG_LIBRARIES})
target_link_libraries(video_encoder_test swscale)
target_link_libraries(video_encoder_test ${OpenCV_LIBS})
target_link_libraries(video_encoder_test yaml-cpp)