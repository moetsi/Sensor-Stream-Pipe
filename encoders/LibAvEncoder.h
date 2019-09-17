//
// Created by amourao on 27-06-2019.
//

#pragma once

#include <fstream>
#include <iostream>
#include <queue>
#include <vector>

#include <cereal/archives/binary.hpp>

#include <yaml-cpp/yaml.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

#include "../readers/ImageReader.h"
#include "../structs/FrameStruct.hpp"
#include "../utils/ImageDecoder.h"
#include "../utils/VideoUtils.h"

#include "IEncoder.h"

class LibAvEncoder : public IEncoder {
private:
  unsigned int totalCurrentFrameCounter;

  AVCodecParameters *pCodecParametersEncoder;
  AVCodecContext *pCodecContextEncoder;
  AVCodec *pCodecEncoder;

  AVFrame *pFrame;
  AVPacket *pPacket;

  CodecParamsStruct *cParamsStruct;

  struct SwsContext *sws_ctx;

  YAML::Node codec_parameters;

  ImageDecoder id;

  std::queue<FrameStruct *> buffer;
  std::queue<AVPacket *> pBuffer;

  uint fps;

  bool ready;

  void init(FrameStruct *fs);

  void encode();

  void encodeA(AVCodecContext *enc_ctx, AVFrame *frame, AVPacket *pkt);

  void prepareFrame();

  std::vector<unsigned char> currentFrameBytes();

public:
  LibAvEncoder(std::string codec_parameters_file, uint _fps);

  LibAvEncoder(YAML::Node &_codec_parameters, uint _fps);

  ~LibAvEncoder();

  void addFrameStruct(FrameStruct *fs);

  void nextPacket();

  bool hasNextPacket();

  FrameStruct *currentFrameEncoded();

  FrameStruct *currentFrameOriginal();

  CodecParamsStruct *getCodecParamsStruct();

  uint getFps();
};
