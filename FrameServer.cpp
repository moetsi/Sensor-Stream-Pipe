//
// Created by amourao on 26-06-2019.
//

#include <ctime>
#include <iostream>
#include <string>
#include <stdlib.h>

#include <asio.hpp>

#include "FrameStruct.hpp"
#include "FrameReader.cpp"
#include "Utils.hpp"

using asio::ip::tcp;

std::string get_frame_message() {
    return getExampleFrameStructBytes();
}

int main(int argc, char *argv[]) {

    try {
        if (argc != 3) {
            std::cerr << "Usage: server <port> <fps>" << std::endl;
            return 1;
        }

        asio::io_context io_context;
        uint port = atoi(argv[1]);
        uint fps = atoi(argv[2]);

        uint64_t last_time = current_time_ms();
        uint64_t start_time = last_time;
        uint64_t sent_frames = 0;

        tcp::acceptor acceptor(io_context, tcp::endpoint(tcp::v4(), port));

        for (;;) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 / fps));
            tcp::socket socket(io_context);
            acceptor.accept(socket);

            if (sent_frames == 0) {
                last_time = current_time_ms();
                start_time = last_time;
            }

            std::string message = get_frame_message();


            asio::error_code ignored_error;
            asio::write(socket, asio::buffer(message), ignored_error);

            socket.close();
            sent_frames += 1;

            uint64_t diff_time = current_time_ms() - last_time;

            double diff_start_time = (current_time_ms() - start_time) / (double) sent_frames;
            int64_t avg_fps;
            if (diff_start_time == 0)
                avg_fps = -1;
            else
                avg_fps = 1000 / diff_start_time;

            std::cout << "Message sent, took " << diff_time << " ms; size " << message.size() << "; avg " << avg_fps << " fps" << std::endl;
            last_time = current_time_ms();


        }
    }
    catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}
