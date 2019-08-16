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

#include "Utils.h"
#include "VideoUtils.h"
#include "SimilarityMeasures.h"


int main(int argc, char *argv[]) {
    srand(time(NULL) * getpid());
    //srand(getpid());

    double psnr = 0;
    cv::Scalar mssim;
    int i = 0;
    int j = 0;

    std::unordered_map<std::string, AVCodec *> pCodecs;
    std::unordered_map<std::string, AVCodecContext *> pCodecContexts;
    std::unordered_map<std::string, AVCodecParameters *> pCodecParameters;

    AVPacket *pPacket = av_packet_alloc();
    AVFrame *pFrame = av_frame_alloc();

    cv::Mat img;
    bool imgChanged = false;

    av_register_all();

    if (argc < 3) {
        std::cerr << "Usage: video_encoder_test <frame_file> <codec parameters>"
                  << std::endl;
        return 1;
    }
    std::string frame_file = std::string(argv[1]);
    std::string codec_parameters_file = std::string(argv[2]);


    FrameReader reader(frame_file);
    FrameEncoder frameEncoder(codec_parameters_file, reader.getFps());


    //This class only reads the file once
    while (reader.hasNextFrame() || frameEncoder.hasNextPacket()) {

        while (!frameEncoder.hasNextPacket()) {
            frameEncoder.addFrameStruct(reader.currentFrame().front());
            reader.nextFrame();
        }

        FrameStruct f = frameEncoder.currentFrame();
        std::vector<FrameStruct> v;
        v.push_back(f);




        if (pCodecs.find(f.streamId) == pCodecs.end()) {
            prepareDecodingStruct(f, pCodecs, pCodecContexts, pCodecParameters);
        }

        AVCodecContext *pCodecContext = pCodecContexts[f.streamId];


        pPacket->data = &f.frame[0];
        pPacket->size = f.frame.size();

        int response = avcodec_send_packet(pCodecContext, pPacket);
        //std::cout << "avcodec_send_packet: " << response << " " << av_err2str(response) << std::endl;
        while (response >= 0) {
            // Return decoded output data (into a frame) from a decoder
            response = avcodec_receive_frame(pCodecContext, pFrame);
            //std::cout << "avcodec_receive_frame: " << response << " " << av_err2str(response) << std::endl;
            if (response == AVERROR(EAGAIN) || response == AVERROR_EOF) {
                break;
            }

            if (response >= 0) {
                i++;

                if (pCodecContext->pix_fmt == AV_PIX_FMT_GRAY12LE) {
                    avframeToMatGray(pFrame, img);
                } else {
                    avframeToMatYUV(pFrame, img);
                }

                imgChanged = true;
            }


        }
        std::cout << f.deviceId << ";" << f.sensorId << ";" << f.frameId << " received; size " << f.frame.size()
                  << std::endl;

        if (imgChanged) {
            FrameStruct fo = frameEncoder.currentFrameOriginal();
            cv::Mat frameOri = cv::imdecode(fo.frame, CV_LOAD_IMAGE_UNCHANGED);
            cv::Mat frameDiff;
            if (pCodecContext->pix_fmt == AV_PIX_FMT_GRAY12LE) {

                cv::Mat frameOriSquached;
                minMaxFilter<ushort>(frameOri, frameOriSquached, 0, MAX_DEPTH_VALUE_12_BITS);
                psnr += getPSNR(frameOriSquached, img, MAX_DEPTH_VALUE_12_BITS);

                frameOriSquached = frameOriSquached * (MAX_DEPTH_VALUE_12_BITS / MAX_DEPTH_VALUE_8_BITS);
                img = img * (MAX_DEPTH_VALUE_12_BITS / MAX_DEPTH_VALUE_8_BITS);

                absdiff(frameOriSquached, img, frameDiff);
                mssim += getMSSIM(frameOriSquached, img);
                frameOri = frameOriSquached;
            } else if (f.frameType == 1) {
                cv::Mat frameOriU = getUMat(frameOri);

                cv::cvtColor(frameOriU, frameOriU, cv::COLOR_GRAY2BGR);

                cv::Mat imgU16;
                cv::cvtColor(img, imgU16, cv::COLOR_BGR2GRAY);
                imgU16.convertTo(imgU16, CV_16UC1);
                imgU16 = imgU16 * (MAX_DEPTH_VALUE_12_BITS / MAX_DEPTH_VALUE_8_BITS);

                cv::Mat frameOriSquached;
                minMaxFilter<ushort>(frameOri, frameOriSquached, 0, MAX_DEPTH_VALUE_12_BITS);

                psnr += getPSNR(frameOriSquached, imgU16, MAX_DEPTH_VALUE_12_BITS);
                mssim += getMSSIM(frameOriSquached, imgU16);

                absdiff(frameOriSquached, imgU16, frameDiff);
                frameOri = frameOriU;

            } else {

                if (frameOri.channels() == 1) {
                    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
                    img.convertTo(img, CV_16UC1);
                }
                absdiff(frameOri, img, frameDiff);
                psnr += getPSNR(frameOri, img, MAX_DEPTH_VALUE_8_BITS);
                mssim += getMSSIM(frameOri, img);
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
        frameEncoder.nextPacket();
    }
    std::cout << frameEncoder.currentFrameId() << " " << i << " " << j << " " << frameEncoder.buffer.size() << " "
              << std::endl;
    std::cout << "Avg PSNR: " << psnr / i << std::endl;
    std::cout << "Avg MSSIM: " << mssim / i << std::endl;
    return 0;
}