//
// Created by amourao on 23-09-2019.
//

#pragma once

#include "zdepth.hpp"
#include <yaml-cpp/yaml.h>
#include <opencv2/imgproc.hpp>

#include "IEncoder.h"
#include "../decoders/LibAvDecoder.h"
#include "../utils/ImageDecoder.h"


class ZDepthEncoder: public IEncoder {
private:
  FrameStruct *frameOriginal;
  FrameStruct *frameCompressed;
  uint totalCurrentFrameCounter;
  uint fps;
  uint width, height;
  zdepth::DepthCompressor compressor;
  LibAvDecoder *fd;
  ImageDecoder id;
  struct SwsContext *sws_ctx;
  std::vector<uint8_t> compressed;
  CodecParamsStruct *paramsStruct;

  std::string stream_id;

public:
  ZDepthEncoder(int _fps);

  ~ZDepthEncoder();

  void addFrameStruct(FrameStruct *fs);

  void nextPacket();

  bool hasNextPacket();

  FrameStruct *currentFrameEncoded();

  FrameStruct *currentFrameOriginal();

  CodecParamsStruct *getCodecParamsStruct();

  uint getFps();

};


