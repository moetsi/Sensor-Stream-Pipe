cmake_minimum_required(VERSION 3.10.2)
project(davp-baseline)

find_package(Threads)
find_package(OpenCV REQUIRED )

#include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(server FrameServer.cpp)
add_executable(client FrameClient.cpp)

target_link_libraries (server ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries (client ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries (server ${OpenCV_LIBS})
target_link_libraries (client ${OpenCV_LIBS})