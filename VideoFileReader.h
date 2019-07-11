//
// Created by amourao on 27-06-2019.
//

#pragma once

#include <iostream>
#include <fstream>
#include <vector>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

#include <cereal/archives/binary.hpp>

#include "FrameStruct.hpp"
#include "Utils.h"


class VideoFileReader {
private:

    bool eofReached;
    unsigned int currentFrameCounter;
    unsigned int fps;
    std::string sceneDesc;
    std::string type;
    unsigned int sensorId;
    AVFormatContext *pFormatContext;
    AVCodecParameters *pCodecParameters;
    AVCodecParameters *pLocalCodecParameters;
    AVCodec *pLocalCodec = NULL;
    AVCodecContext *pCodecContext;
    int video_stream_index;
    AVFrame *pFrame;
    AVPacket *pPacket;
    AVCodec *pCodec;

    unsigned int deviceId;

    std::vector<std::string> frameLines;

    std::vector<unsigned char> readFile(std::string &filename);

    int decode_packet();

public:
    VideoFileReader(std::string filename);

    ~VideoFileReader();

    void reset();

    void goToFrame(unsigned int frameId);

    bool hasNextFrame();

    void nextFrame();

    std::vector<unsigned char> currentFrameBytes();

    unsigned int currentFrameId();

    std::string getSceneDesc();

    unsigned int getFps();

    unsigned int getSensorId();

    unsigned int getDeviceId();


};


