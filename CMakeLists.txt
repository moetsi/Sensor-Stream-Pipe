cmake_minimum_required(VERSION 3.10.2)
project(davp-baseline)

find_package(Threads)
find_package(OpenCV REQUIRED )

include_directories("$ENV{HOME}/libs/include")

set(SOURCE_FILES FrameReader.cpp FrameStruct.hpp)

add_executable(server FrameServer.cpp ${SOURCE_FILES})
add_executable(client FrameClient.cpp ${SOURCE_FILES})

target_link_libraries (server ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries (client ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries (server ${OpenCV_LIBS})
target_link_libraries (client ${OpenCV_LIBS})