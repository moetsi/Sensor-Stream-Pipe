//
// Created by amourao on 27-06-2019.
//

#pragma once

#include <iostream>
#include <fstream>
#include <vector>

#include <cereal/archives/binary.hpp>

#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>
}

#include "FrameStruct.hpp"
#include "FrameReader.h"

class FrameEncoder : public FrameReader {
private:

    unsigned int totalCurrentFrameCounter;
    std::string codec_info_string;

    AVFormatContext *pFormatContext;
    AVCodecParameters *pCodecParameters;
    AVCodecContext *pCodecContext;
    AVCodec *pCodec;

    AVFrame *pFrame;
    AVPacket *pPacket;

    bool libAVReady;
    std::string streamId;

    void init();

    void encode();

public:
    FrameEncoder(std::string filename, std::string frame_filename);

    ~FrameEncoder();

    void goToFrame(unsigned int frameId);

    void nextFrame();

    std::vector<unsigned char> currentFrameBytes();

    unsigned int currentFrameId();

    CodecParamsStruct getCodecParamsStruct();

    std::string getStreamID();
};


