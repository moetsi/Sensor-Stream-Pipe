//
// Created by amourao on 27-06-2019.
//

#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <queue>

#include <cereal/archives/binary.hpp>

#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <cv.hpp>

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

class FrameEncoder : public FrameReader {
private:

    unsigned int totalCurrentFrameCounter;
    unsigned int currentFrameCounterReset;
    unsigned int encoderCalls;


    AVCodecParameters *pCodecParameters;
    AVCodecContext *pCodecContext;
    AVCodec *pCodec;

    AVFrame *pFrame;
    AVPacket *pPacket;

    YAML::Node codec_parameters;

    unsigned int bufferSize;
    bool needsToBreak;
    std::string streamId;

    void init();

    void encode();

    void encodeA(AVCodecContext *enc_ctx, AVFrame *frame, AVPacket *pkt);

    void prepareFrame();

public:

    std::queue<std::vector<FrameStruct>> buffer;
    std::queue<AVPacket> pBuffer;

    FrameEncoder(std::string filename, std::string frame_filename);

    ~FrameEncoder();

    void nextFrame();

    bool hasNextFrame();

    std::vector<FrameStruct> currentFrame();

    void reset();

    std::vector<unsigned char> currentFrameBytes(AVPacket &packet);

    unsigned int currentFrameId();

    CodecParamsStruct getCodecParamsStruct();

    std::string getStreamID();

    FrameStruct currentFrameVid();

    std::vector<FrameStruct> currentFrameSync();
};
