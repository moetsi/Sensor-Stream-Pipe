//
// Created by amourao on 26-06-2019.
//

#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>

#include <zmq.hpp>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}


#include "../structs/FrameStruct.hpp"
#include "../readers/FrameReader.h"
#include "../utils/Utils.h"
#include "../utils/VideoUtils.h"

int main(int argc, char *argv[]) {

    srand(time(NULL) * getpid());

    try {
        if (argc != 2) {
            std::cerr << "Usage: client <port>" << std::endl;
            return 1;
        }
        zmq::context_t context(1);
        zmq::socket_t socket(context, ZMQ_PULL);
        socket.bind("tcp://*:" + std::string(argv[1]));

        uint64_t last_time = currentTimeMs();
        uint64_t start_time = last_time;
        uint64_t rec_frames = 0;
        double rec_mbytes = 0;

        std::unordered_map<std::string, AVCodec *> pCodecs;
        std::unordered_map<std::string, AVCodecContext *> pCodecContexts;
        std::unordered_map<std::string, AVCodecParameters *> pCodecParameters;

        AVPacket *pPacket = av_packet_alloc();
        AVFrame *pFrame = av_frame_alloc();
        if (!pFrame) {
            std::cout << ("failed to allocated memory for AVFrame") << std::endl;
            return -1;
        }
        // https://ffmpeg.org/doxygen/trunk/structAVPacket.html

        if (!pPacket) {
            std::cout << ("failed to allocated memory for AVPacket") << std::endl;
            return -1;
        }

        av_register_all();


        bool imgChanged = false;

        for (;;) {
            zmq::message_t request;

            socket.recv(&request);

            if (rec_frames == 0) {
                last_time = currentTimeMs();
                start_time = last_time;
            }

            rec_frames += 1;
            uint64_t diff_time = currentTimeMs() - last_time;
            double diff_start_time = (currentTimeMs() - start_time) / (double) rec_frames;
            int64_t avg_fps;
            if (diff_start_time == 0)
                avg_fps = -1;
            else
                avg_fps = 1000 / diff_start_time;

            last_time = currentTimeMs();

            std::string result = std::string(static_cast<char *>(request.data()), request.size());

            std::vector<FrameStruct> f_list = parseCerealStructFromString<std::vector<FrameStruct>>(result);
            rec_mbytes += request.size() / 1000;

            for (FrameStruct f: f_list) {
                cv::Mat img;
                if (f.messageType == 0) {
                    img = cv::imdecode(f.frame, CV_LOAD_IMAGE_UNCHANGED);
                    imgChanged = true;
                } else if (f.messageType == 1) {

                    if (pCodecs.find(f.streamId) == pCodecs.end()) {
                        prepareDecodingStruct(f, pCodecs, pCodecContexts, pCodecParameters);
                    }


                    AVCodecContext *pCodecContext = pCodecContexts[f.streamId];

                    pPacket->data = &f.frame[0];
                    pPacket->size = f.frame.size();

                    if (f.frameId == 1) { // reset the codec context on video reset
                        std::cout << "Resetting stream" << std::endl;
                        avcodec_flush_buffers(pCodecContexts[f.streamId]);
                    }


                    int response = avcodec_send_packet(pCodecContext, pPacket);
                    if (response >= 0) {
                        // Return decoded output data (into a frame) from a decoder
                        response = avcodec_receive_frame(pCodecContext, pFrame);
                        if (response >= 0) {
                            if (pCodecContext->pix_fmt == AV_PIX_FMT_GRAY12LE) {
                                avframeToMatGray(pFrame, img);
                            } else {
                                avframeToMatYUV(pFrame, img);
                            }
                            imgChanged = true;
                        } else {
                            std::cerr << "avcodec_receive_frame: " << av_err2str(response) << std::endl;
                        }
                    } else {
                        std::cerr << "avcodec_send_packet: " << av_err2str(response) << std::endl;
                    }

                    f.frame.clear();

                }

                if (imgChanged && !img.empty()) {
                    cv::namedWindow(f.streamId + std::to_string(f.sensorId));
                    cv::imshow(f.streamId + std::to_string(f.sensorId), img);
                    cv::waitKey(1);
                    imgChanged = false;
                }

                std::cout << f.deviceId << ";" << f.sensorId << ";" << f.frameId << " received, took " << diff_time
                      << " ms; size " << request.size()
                      << "; avg " << avg_fps << " fps; " << 8 * (rec_mbytes / (currentTimeMs() - start_time))
                      << " Mbps" << std::endl;
            }
        }

    }
    catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
    }

    //avformat_close_input(&pFormatContext);
    //avformat_free_context(pFormatContext);
    //av_packet_free(&pPacket);
    //av_frame_free(&pFrame);
    //avcodec_free_context(&pCodecContext);

    return 0;
}


