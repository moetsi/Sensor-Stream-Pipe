//
// Created by amourao on 07/08/19.
//

#pragma once

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

#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>

#include "../structs/FrameStruct.hpp"

void avframeToMat(const AVFrame *frame, cv::Mat &image) {
    int width = frame->width;
    int height = frame->height;

    //std::cout << av_frame_get_color_range(frame) << " " << image.step1() << " " << image.step1(0) << std::endl;

    SwsContext *conversion;

    //TODO: this only works for 3 channel 8 bit color frames, make it work also for 1 channel 16 bit depth
    // Allocate the opencv mat and store its stride in a 1-element array
    image = cv::Mat(height, width, CV_8UC3);
    conversion = sws_getContext(width, height, (AVPixelFormat) frame->format, width, height, AV_PIX_FMT_BGR24,
                                SWS_FAST_BILINEAR, NULL, NULL, NULL);
    int cvLinesizes[1];
    cvLinesizes[0] = image.step1();

    // Convert the colour format and write directly to the opencv matrix
    sws_scale(conversion, frame->data, frame->linesize, 0, height, &image.data, cvLinesizes);
    sws_freeContext(conversion);
}

void prepareDecodingStruct(FrameStruct &f, std::unordered_map<std::string, AVCodec *> &pCodecs,
                           std::unordered_map<std::string, AVCodecContext *> &pCodecContexts,
                           std::unordered_map<std::string, AVCodecParameters *> &pCodecParameters) {
    AVCodecParameters *pCodecParameter = f.codec_data.getParams();
    AVCodec *pCodec = avcodec_find_decoder(f.codec_data.getParams()->codec_id);
    AVCodecContext *pCodecContext = avcodec_alloc_context3(pCodec);

    if (!pCodecContext) {
        std::cerr << "failed to allocated memory for AVCodecContext" << std::endl;
        exit(1);
    }

    if (avcodec_parameters_to_context(pCodecContext, pCodecParameter) < 0) {
        std::cerr << ("failed to copy codec params to codec context") << std::endl;
        exit(1);
    }

    if (avcodec_open2(pCodecContext, pCodec, NULL) < 0) {
        std::cerr << ("failed to open codec through avcodec_open2") << std::endl;
        exit(1);
    }

    pCodecs[f.streamId] = pCodec;
    pCodecContexts[f.streamId] = pCodecContext;
    pCodecParameters[f.streamId] = pCodecParameter;
}

