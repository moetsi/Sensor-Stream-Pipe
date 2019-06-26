//
// Created by amourao on 26-06-2019.
//

#include <iostream>
#include <asio.hpp>

#include <chrono>
#include <thread>

#include "FrameStruct.hpp"
#include "FrameReader.cpp"
#include "Utils.hpp"

using asio::ip::tcp;

#define BUFFER_SIZE 1610610

int main(int argc, char *argv[]) {
    try {
        if (argc != 3) {
            std::cerr << "Usage: client <host> <port>" << std::endl;
            return 1;
        }

        asio::io_context io_context;

        tcp::resolver resolver(io_context);
        tcp::resolver::results_type endpoints =
                resolver.resolve(argv[1], argv[2]);

        tcp::socket socket(io_context);
        asio::error_code error = asio::error::host_unreachable;
        asio::streambuf streamBuffer;
        asio::streambuf::mutable_buffers_type mutableBuffer =
                streamBuffer.prepare(BUFFER_SIZE);

        uint64_t last_time = current_time_ms();
        uint64_t start_time = last_time;
        uint64_t rec_frames = 0;
        for (;;) {
            while (error)
                asio::connect(socket, endpoints, error);

            if (rec_frames == 0) {
                last_time = current_time_ms();
                start_time = last_time;
            }

            for (;;) {
                size_t len = socket.read_some(asio::buffer(mutableBuffer), error);
                streamBuffer.commit(len);

                if (error == asio::error::eof) {
                    rec_frames += 1;
                    uint64_t diff_time = current_time_ms() - last_time;
                    double diff_start_time = (current_time_ms() - start_time) / (double) rec_frames;
                    int64_t avg_fps;
                    if (diff_start_time == 0)
                        avg_fps = -1;
                    else
                        avg_fps = 1000 / diff_start_time;

                    std::cout << "EOF received, took " << diff_time << " ms; avg " << avg_fps << " fps" << std::endl;
                    last_time = current_time_ms();
                    FrameStruct f = parseFrameStruct(streamBuffer);
                    //streamBuffer.consume(streamBuffer.in_avail());
                    break; // Connection closed cleanly by peer.
                } else if (error) {
                    throw asio::system_error(error); // Some other error.
                }
            }


        }
    }
    catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}
