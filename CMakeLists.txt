cmake_minimum_required(VERSION 3.10.2)
project(davp-baseline)

find_package(Threads)

add_executable(server FrameServer.cpp)
add_executable(client FrameClient.cpp)

target_link_libraries (server ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries (client ${CMAKE_THREAD_LIBS_INIT})