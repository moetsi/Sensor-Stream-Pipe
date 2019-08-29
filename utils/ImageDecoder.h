//
// Created by amourao on 28/08/19.
//

#pragma once

#include <vector>
#include <fstream>
#include <iostream>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavformat/avio.h>
#include <libavutil/file.h>
};

#include "Utils.h"

class ImageDecoder {

private:


    AVFormatContext *pFormatContext = NULL;
    AVIOContext *avio_ctx = NULL;
    AVCodecParameters *pCodecParameters;
    AVCodecContext *pCodecContext;
    AVCodec *pCodec;
    AVPacket *pPacket;

    uint8_t *avio_ctx_buffer = NULL;
    size_t avio_ctx_buffer_size = 4096;

    bool libAVReady;

    void init(std::vector<unsigned char> &buffer);

    int decode_packet(AVFrame *pFrame);

public:

    ImageDecoder();

    ~ImageDecoder();


    int getWidth();

    int getHeigth();

    void imageBufferToAVFrame(std::vector<unsigned char> &buffer, AVFrame *pFrame);


};
