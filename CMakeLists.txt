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

set(SOURCE_FILES FrameReader.cpp FrameStruct.hpp Utils.cpp)

add_executable(client FrameProcessingClient.cpp VideoFileReader.cpp ${SOURCE_FILES})
add_executable(frame_server FrameServer.cpp ${SOURCE_FILES})
add_executable(video_file_server VideoFileServer.cpp VideoFileReader.cpp ${SOURCE_FILES})
add_executable(video_encoder_server VideoEncoderServer.cpp FrameEncoder.cpp ${SOURCE_FILES})


target_link_libraries(client swscale)
target_link_libraries(client zmq)
target_link_libraries(client ${FFMPEG_LIBRARIES})
target_link_libraries(client ${OpenCV_LIBS})

target_link_libraries(frame_server zmq)
target_link_libraries(frame_server ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(frame_server ${OpenCV_LIBS})

target_link_libraries(video_file_server swscale)
target_link_libraries(video_file_server zmq)
target_link_libraries(video_file_server ${FFMPEG_LIBRARIES})
target_link_libraries(video_file_server ${OpenCV_LIBS})

target_link_libraries(video_encoder_server swscale)
target_link_libraries(video_encoder_server zmq)
target_link_libraries(video_encoder_server ${FFMPEG_LIBRARIES})
target_link_libraries(video_encoder_server ${OpenCV_LIBS})
target_link_libraries(video_encoder_server yaml-cpp)
