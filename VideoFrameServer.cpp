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
#include "FrameEncoder.h"
#include "Utils.h"


int main(int argc, char *argv[]) {

    srand(time(NULL) * getpid());
    //srand(getpid());

    try {
        if (argc < 5) {
            std::cerr << "Usage: server <host> <port> <frame_file>"
                      << std::endl;
            return 1;
        }

        zmq::context_t context(1);
        zmq::socket_t socket(context, ZMQ_PUSH);


        std::string host = std::string(argv[1]);
        uint port = std::stoul(argv[2]);

        std::string frame_file = std::string(argv[3]);
        std::string codec_name = std::string(argv[4]);

        int stopAfter = INT_MAX;
        if (argc >= 6) {
            stopAfter = std::stoi(argv[5]);
        }

        FrameEncoder fc(frame_file, "libx264");

        uint fps = fc.getFps();


        uint64_t last_time = currentTimeMs();
        uint64_t start_time = last_time;
        uint64_t start_frame_time = last_time;
        uint64_t sent_frames = 0;
        uint64_t processing_time = 0;

        double sent_mbytes = 0;

        socket.connect("tcp://" + host + ":" + std::string(argv[2]));

        while (stopAfter > 0) {
            // try to maintain constant FPS by ignoring processing time
            uint64_t sleep_time = (1000 / fps) - processing_time;

            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
            start_frame_time = currentTimeMs();

            fc.nextFrame();

            VideoFrameStruct f;

            if (sent_frames == 0) {
                last_time = currentTimeMs();
                start_time = last_time;
            }
            f.codec_data.push_back(fc.getCodecParamsStruct());

            f.messageType = 1;

            f.frames.push_back(fc.currentFrameBytes());

            f.frameId = fc.currentFrameId();
            f.streamId = fc.getStreamID();

            if (!fc.hasNextFrame()) {
                fc.reset();
                stopAfter--;
            }

            std::string message = serialize_to_str(f);

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
