//
// Created by amourao on 26-06-2019.
//

#include <unistd.h>

#include <k4a/k4a.h>

#include <ctime>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <thread>

#include <zmq.hpp>

#include "../structs/FrameStruct.hpp"
#include "../readers/KinectReader.h"
#include "../utils/Utils.h"

int main(int argc, char *argv[]) {

    srand(time(NULL) * getpid());

    try {


        if (argc < 3) {
            std::cerr << "Usage: server <host> <port>" << std::endl;
            return 1;
        }

        zmq::context_t context(1);
        zmq::socket_t socket(context, ZMQ_PUSH);


        std::string host = std::string(argv[1]);
        uint port = std::stoul(argv[2]);

        //TODO: Read parameters from file
        k4a_image_format_t recording_color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
        k4a_color_resolution_t recording_color_resolution = K4A_COLOR_RESOLUTION_2160P;
        k4a_depth_mode_t recording_depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
        k4a_fps_t recording_rate = K4A_FRAMES_PER_SECOND_30;

        k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        device_config.color_format = recording_color_format;
        device_config.color_resolution = recording_color_resolution;
        device_config.depth_mode = recording_depth_mode;
        device_config.camera_fps = recording_rate;
        device_config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
        device_config.depth_delay_off_color_usec = 0;
        device_config.subordinate_delay_off_master_usec = 0;


        KinectReader reader(0, &device_config, 0);

        uint64_t last_time = currentTimeMs();
        uint64_t start_time = last_time;
        uint64_t start_frame_time = last_time;
        uint64_t sent_frames = 0;
        uint64_t processing_time = 0;

        double sent_mbytes = 0;


        socket.connect("tcp://" + host + ":" + std::string(argv[2]));

        while (1) {
            // maintain constant FPS by ignoring processing time
            start_frame_time = currentTimeMs();

            if (sent_frames == 0) {
                last_time = currentTimeMs();
                start_time = last_time;
            }

            std::vector<FrameStruct> v = reader.currentFrame();


            if (reader.hasNextFrame())
                reader.nextFrame();
            else {
                reader.reset();
            }

            if (!v.empty()) {

                std::string message = cerealStructToString(v);

                zmq::message_t request(message.size());
                memcpy(request.data(), message.c_str(), message.size());

                socket.send(request);
                sent_frames += 1;
                sent_mbytes += message.size() / 1000.0;

                uint64_t diff_time = currentTimeMs() - last_time;

                double diff_start_time = (currentTimeMs() - start_time);
                int64_t avg_fps;
                if (diff_start_time == 0)
                    avg_fps = -1;
                else {
                    double avg_time_per_frame_sent_ms = diff_start_time / (double) sent_frames;
                    avg_fps = 1000 / avg_time_per_frame_sent_ms;
                }

                last_time = currentTimeMs();
                processing_time = last_time - start_frame_time;

                std::cout << "Took " << diff_time << " ms; size " << message.size()
                          << "; avg " << avg_fps << " fps; " << 8 * (sent_mbytes / diff_start_time) << " Mbps"
                          << std::endl;
                for (uint i = 0; i < v.size(); i++) {
                    FrameStruct f = v.at(i);
                    std::cout << "\t" << f.deviceId << ";" << f.sensorId << ";" << f.frameId << " sent" << std::endl;
                }

            }

        }
    }
    catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}
