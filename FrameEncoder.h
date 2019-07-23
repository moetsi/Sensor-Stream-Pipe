//
// Created by amourao on 27-06-2019.
//

#pragma once

#include <iostream>
#include <fstream>
#include <vector>

#include <cereal/archives/binary.hpp>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

#include "FrameStruct.hpp"
#include "FrameReader.h"

class FrameEncoder : public FrameReader {
private:
    AVFormatContext *pFormatContext;
    AVCodecParameters *pCodecParameters;
    AVCodecContext *pCodecContext;
    AVCodec *pCodec;

    AVFrame *pFrame;
    AVPacket *pPacket;

    bool libAVReady;

public:
    FrameEncoder(std::string filename, std::string frame_filename);

    void reset();

    void goToFrame(unsigned int frameId);

    void nextFrame();

    FrameStruct currentFrame();

    std::string currentFrameBytes();
};


