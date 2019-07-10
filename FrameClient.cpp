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
        double rec_mbytes = 0;


        for (;;) {



            zmq::message_t request;

            socket.recv(&request);

            if (rec_frames == 0) {
                last_time = current_time_ms();
                start_time = last_time;
            }

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
            //TODO: check if it is also necessary to copy from zeromq buffer
            std::string result = std::string(static_cast<char *>(request.data()), request.size());

            FrameStruct f = FrameStruct::parseFrameStruct(result);
            rec_mbytes += request.size()/1000;


            cv::Mat color = f.getColorFrame();
            cv::Mat depth = f.getDepthFrame();
            cv::namedWindow("Display Window");
            cv::imshow("Display Window", color);
            cv::waitKey(1);
            std::cout << f.deviceId << ";" << f.sensorId << ";" << f.frameId << " received, took " << diff_time << " ms; size " << request.size()
                      << "; avg " << avg_fps << " fps; " << 8*(rec_mbytes/(current_time_ms()-start_time)) << " Mbps" << std::endl;

        }
    }
    catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}
