//
// Created by amourao on 27-06-2019.
//

#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <queue>

#include <cereal/archives/binary.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>


#include <yaml-cpp/yaml.h>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>
}

#include "../structs/FrameStruct.hpp"
#include "../readers/FrameReader.h"
#include "../utils/VideoUtils.h"

class FrameEncoder {
private:

    unsigned int totalCurrentFrameCounter;

    AVCodecParameters *pCodecParameters;
    AVCodecContext *pCodecContext;
    AVCodec *pCodec;

    AVFrame *pFrame;
    AVPacket *pPacket;


    cv::Mat frameOri;

    YAML::Node codec_parameters;

    uint fps;

    bool ready;

    void init(FrameStruct fs);

    void encode();

    void encodeA(AVCodecContext *enc_ctx, AVFrame *frame, AVPacket *pkt);

    void prepareFrame();

    std::vector<unsigned char> currentFrameBytes(AVPacket &packet);

public:

    std::queue<FrameStruct> buffer;
    std::queue<AVPacket> pBuffer;

    FrameEncoder(std::string codec_parameters_file, uint fps);

    ~FrameEncoder();

    void addFrameStruct(FrameStruct &fs);

    void nextPacket();

    bool hasNextPacket();

    FrameStruct currentFrame();

    FrameStruct currentFrameOriginal();

    CodecParamsStruct getCodecParamsStruct();

    unsigned int currentFrameId();

    uint getFps();



};
