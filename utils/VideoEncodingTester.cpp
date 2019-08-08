//
// Created by amourao on 07/08/19.
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


#include "../encoders/FrameEncoder.h"
#include "../structs/FrameStruct.hpp"
#include "../readers/FrameReader.h"
#include "../utils/Utils.h"
#include "VideoUtils.h"


int main(int argc, char *argv[]) {
    srand(time(NULL) * getpid());
    //srand(getpid());

    double psnr = 0;
    int i = 0;

    std::unordered_map<std::string, AVCodec *> pCodecs;
    std::unordered_map<std::string, AVCodecContext *> pCodecContexts;
    std::unordered_map<std::string, AVCodecParameters *> pCodecParameters;

    AVPacket *pPacket = av_packet_alloc();
    AVFrame *pFrame = av_frame_alloc();

    cv::Mat img;
    bool imgChanged = false;

    av_register_all();



    try {
        if (argc < 3) {
            std::cerr << "Usage: video_encoder_test <frame_file> <codec parameters>"
                      << std::endl;
            return 1;
        }
        std::string frame_file = std::string(argv[1]);
        std::string codec_parameters_file = std::string(argv[2]);


        FrameEncoder fc(frame_file, codec_parameters_file);
        FrameReader fr(frame_file);

        //This class only reads the file once
        while (fc.hasNextFrame()) {
            FrameStruct fo = fr.currentFrame();
            FrameStruct f = fc.currentFrameVid();

            cv::Mat frameOri = cv::imdecode(fo.frame, CV_LOAD_IMAGE_UNCHANGED);

            double localMin, localMax;
            cv::minMaxLoc(frameOri, &localMin, &localMax);

            if (pCodecs.find(f.streamId) == pCodecs.end()) {
                prepareDecodingStruct(f, pCodecs, pCodecContexts, pCodecParameters);
            }

            AVCodecContext *pCodecContext = pCodecContexts[f.streamId];

            // reset the codec context on restarting the video
            if (fc.currentFrameId() == 1) {
                std::cout << "Resetting stream" << std::endl;
                avcodec_flush_buffers(pCodecContext);
            }

            pPacket->data = &f.frame[0];
            pPacket->size = f.frame.size();

            int response = avcodec_send_packet(pCodecContext, pPacket);
            if (response >= 0) {
                // Return decoded output data (into a frame) from a decoder
                response = avcodec_receive_frame(pCodecContext, pFrame);
                if (response >= 0) {
                    i++;
                    avframeToMat(pFrame, img);
                    imgChanged = true;
                    fr.nextFrame();
                }
            }

            if (imgChanged) {
                cv::Mat frameDiff;

                if (fc.getFrameType() == 1) {
                    cv::Mat frameOriU = getUMat(frameOri);

                    cv::cvtColor(frameOriU, frameOriU, cv::COLOR_GRAY2BGR);

                    cv::Mat imgU16;
                    cv::cvtColor(img, imgU16, cv::COLOR_BGR2GRAY);
                    imgU16.convertTo(imgU16, CV_16UC1);
                    imgU16 = imgU16 * (MAX_DEPTH_VALUE / 256);

                    cv::Mat frameOriSquached;
                    minMaxFilter<ushort>(frameOri, frameOriSquached, 0, 4096);

                    psnr += getPSNR(frameOriSquached, imgU16, 4096);
                    absdiff(frameOri, imgU16, frameDiff);
                    frameOri = frameOriU;
                } else {
                    absdiff(frameOri, img, frameDiff);
                    psnr += getPSNR(frameOri, img, 256);
                }

                cv::namedWindow("Original");
                cv::imshow("Original", frameOri);
                cv::namedWindow("Encoded");
                cv::imshow("Encoded", img);
                cv::namedWindow("Diff");

                cv::imshow("Diff", frameDiff);

                cv::waitKey(1);
                imgChanged = false;
            }

            fc.nextFrame();
        }
    }
    catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
    }
    std::cout << psnr / i << std::endl;
    return 0;
}