//
// Created by amourao on 28/08/19.
//

#pragma once

#include <fstream>
#include <iostream>
#include <vector>

#include <spdlog/spdlog.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavformat/avio.h>
#include <libavutil/file.h>
};

#include "../structs/FrameStruct.hpp"
#include "Utils.h"

class ImageDecoder {

private:
  AVFormatContext *pFormatContext = NULL;
  AVIOContext *avio_ctx = NULL;
  AVCodecParameters *pCodecParameters;
  AVCodecContext *pCodecContext;
  AVCodec *pCodec;
  AVPacket *pPacket;

  CodecParamsStruct *cParamsStruct;

  uint8_t *avio_ctx_buffer = NULL;
  size_t avio_ctx_buffer_size = 4096;

  bool libAVReady;

  void init(std::vector<unsigned char> &buffer);

  int decode_packet(AVFrame *pFrame);
  CodecParamsStruct *getCodecParamsStruct();

public:
  ImageDecoder();

  ~ImageDecoder();

  void imageBufferToAVFrame(FrameStruct *fs, AVFrame *pFrame);
};
