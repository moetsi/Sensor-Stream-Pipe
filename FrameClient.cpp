//
// Created by amourao on 26-06-2019.
//

#include <iostream>
#include <chrono>
#include <thread>

#include <zmq.hpp>

#include "FrameStruct.hpp"
#include "FrameReader.h"
#include "Utils.hpp"

using asio::ip::tcp;

#define BUFFER_SIZE 1610610

int main(int argc, char *argv[]) {
    try {
        if (argc != 2) {
            std::cerr << "Usage: client <port>" << std::endl;
            return 1;
        }
        zmq::context_t context(1);
        zmq::socket_t socket(context, ZMQ_PULL);
        socket.bind("tcp://*:" + std::string(argv[1]));

        //socket.setsockopt(ZMQ_SUBSCRIBE, nullptr, 0);

        uint64_t last_time = current_time_ms();
        uint64_t start_time = last_time;
        uint64_t rec_frames = 0;
        size_t paclet_len = 0;
        for (;;) {

            if (rec_frames == 0) {
                last_time = current_time_ms();
                start_time = last_time;
            }

            zmq::message_t request;

            socket.recv(&request);

            rec_frames += 1;
            uint64_t diff_time = current_time_ms() - last_time;
            double diff_start_time = (current_time_ms() - start_time) / (double) rec_frames;
            int64_t avg_fps;
            if (diff_start_time == 0)
                avg_fps = -1;
            else
                avg_fps = 1000 / diff_start_time;

                    //std::cout << "Message received, took " << diff_time << " ms; size " << paclet_len << "; avg " << avg_fps << " fps" << std::endl;

            last_time = current_time_ms();

            //std::string result = request.str();
            std::string result = std::string(static_cast<char *>(request.data()), request.size());

            FrameStruct f = FrameStruct::parseFrameStruct(result);
            //cv::Mat color = f.getColorFrame();
            //cv::Mat depth = f.getDepthFrame();
            std::cout << "Frame " << f.frameId << " received, took " << diff_time << " ms; size " << request.size()
                      << "; avg " << avg_fps << " fps" << std::endl;

        }
    }
    catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}
