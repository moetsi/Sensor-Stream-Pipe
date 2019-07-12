//
// Created by amourao on 26-06-2019.
//

#include <ctime>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <thread>

#include <zmq.hpp>

#include "FrameStruct.hpp"
#include "VideoFileReader.h"
#include "Utils.h"

int main(int argc, char *argv[]) {

    try {
        if (argc < 5) {
            std::cerr << "Usage: server <host> <port> <color video file> <depth video file> (<stop after>)"
                      << std::endl;
            return 1;
        }

        zmq::context_t context(1);
        zmq::socket_t socket(context, ZMQ_PUSH);


        std::string host = std::string(argv[1]);
        uint port = std::stoul(argv[2]);

        std::string color_video_name = std::string(argv[3]);
        std::string depth_video_name = std::string(argv[4]);

        int stopAfter = INT_MAX;
        if (argc >= 6) {
            stopAfter = std::stoi(argv[5]);
        }

        VideoFileReader vc(color_video_name);
        VideoFileReader vd(depth_video_name);

        uint fps = vc.getFps();


        uint64_t last_time = current_time_ms();
        uint64_t start_time = last_time;
        uint64_t start_frame_time = last_time;
        uint64_t sent_frames = 0;
        uint64_t processing_time = 0;

        //TODO: make sure all the units are OK and match the conversion to Mbps
        double sent_mbytes = 0;

        socket.connect("tcp://" + host + ":" + std::string(argv[2]));

        while (stopAfter > 0) {
            // try to maintain constant FPS by ignoring processing time
            uint64_t sleep_time = (1000 / fps) - processing_time;

            //TODO: see if the if is needed
            if (sleep_time >= 1)
                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
            start_frame_time = current_time_ms();

            if (sent_frames == 0) {
                last_time = current_time_ms();
                start_time = last_time;
            }

            FrameStruct f;

            vc.nextFrame();
            vd.nextFrame();

            f.messageType = 1;

            f.colorFrame = vc.currentFrameBytes();
            f.depthFrame = vd.currentFrameBytes();

            f.frameId = vc.currentFrameId();

            if (!vc.hasNextFrame()) {
                vc.reset();
                stopAfter--;
            }

            if (!vd.hasNextFrame()) {
                vd.reset();
            }

            std::string message = FrameStruct::getStructBytes(f);


            zmq::message_t request(message.size());
            memcpy(request.data(), message.c_str(), message.size());

            socket.send(request);
            sent_frames += 1;
            sent_mbytes += message.size() / 1000.0;

            uint64_t diff_time = current_time_ms() - last_time;

            double diff_start_time = (current_time_ms() - start_time);
            int64_t avg_fps;
            if (diff_start_time == 0)
                avg_fps = -1;
            else
                //TODO: detail the conversions happening here
                avg_fps = 1000 / (diff_start_time / (double) sent_frames);

            last_time = current_time_ms();
            processing_time = last_time - start_frame_time;

            std::cout << f.deviceId << ";" << f.sensorId << ";" << f.frameId << " sent, took " << diff_time
                      << " ms; size " << message.size()
                      << "; avg " << avg_fps << " fps; " << 8 * (sent_mbytes / diff_start_time) << " Mbps" << std::endl;


        }
    }
    catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}
