//
// Created by amourao on 26-06-2019.
//

#include <ctime>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <thread>
#include <unistd.h>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}


#include <zmq.hpp>

#include "FrameStruct.hpp"
#include "VideoFileReader.h"
#include "Utils.h"


int main(int argc, char *argv[]) {

    srand(time(NULL) * getpid());
    //srand(getpid());

    try {
        if (argc < 5) {
            std::cerr << "Usage: server <host> <port> <video file> <video type> (<stop after>)"
                      << std::endl;
            return 1;
        }

        zmq::context_t context(1);
        zmq::socket_t socket(context, ZMQ_PUSH);


        std::string host = std::string(argv[1]);
        uint port = std::stoul(argv[2]);

        std::string video_name = std::string(argv[3]);

        int video_type = std::stoul(argv[4]);

        int stopAfter = INT_MAX;
        if (argc >= 6) {
            stopAfter = std::stoi(argv[5]);
        }

        VideoFileReader videoReader(video_name);

        uint fps = videoReader.getFps();


        uint64_t last_time = currentTimeMs();
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

            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
            start_frame_time = currentTimeMs();

            videoReader.nextFrame();


            FrameStruct frameStruct;

            if (sent_frames == 0) {
                last_time = currentTimeMs();
                start_time = last_time;
            }
            frameStruct.codec_data = videoReader.getCodecParamsStruct();
            frameStruct.messageType = 1;
            frameStruct.frame = videoReader.currentFrameBytes();
            frameStruct.frameId = videoReader.currentFrameId();
            frameStruct.streamId = videoReader.getStreamID();
            frameStruct.deviceId = videoReader.getDeviceId();
            frameStruct.sensorId = videoReader.getSensorId();
            frameStruct.frameType = video_type;


            if (!videoReader.hasNextFrame()) {
                videoReader.reset();
                stopAfter--;
            }

            std::string message = cerealStructToString(frameStruct);
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
            else
                //TODO: detail the conversions happening here
                avg_fps = 1000 / (diff_start_time / (double) sent_frames);

            last_time = currentTimeMs();
            processing_time = last_time - start_frame_time;

            std::cout << frameStruct.deviceId << ";" << frameStruct.sensorId << ";" << frameStruct.frameId
                      << " sent, took " << diff_time
                      << " ms; size " << message.size()
                      << "; avg " << avg_fps << " fps; " << 8 * (sent_mbytes / diff_start_time) << " Mbps" << std::endl;

        }
    }
    catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}
