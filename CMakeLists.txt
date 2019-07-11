cmake_minimum_required(VERSION 3.10.2)
project(davp-baseline)

include_directories("$ENV{HOME}/libs/include")
link_directories("$ENV{HOME}/libs/lib")

find_package(Threads)
find_package(OpenCV REQUIRED)

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

set(SOURCE_FILES FrameReader.cpp FrameStruct.hpp)

add_executable(server FrameServer.cpp ${SOURCE_FILES})
add_executable(client FrameClient.cpp ${SOURCE_FILES})
add_executable(video VideoMain.cpp)

target_link_libraries(server zmq)
target_link_libraries(server ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(server ${OpenCV_LIBS})

target_link_libraries(client zmq)
target_link_libraries(client ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(client ${OpenCV_LIBS})

target_link_libraries(video swscale)
target_link_libraries(video ${FFMPEG_LIBRARIES})
target_link_libraries(video ${OpenCV_LIBS})