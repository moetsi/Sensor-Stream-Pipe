//
// Created by amourao on 11-09-2019.
//

#pragma once

#include "../decoders/LibAvDecoder.h"
#include "../utils/ImageDecoder.h"
#include "IEncoder.h"
#include <NvPipe.h>
#include <yaml-cpp/yaml.h>

class NvEncoder : public IEncoder {

private:
  NvPipe *encoder;
  std::vector<uint8_t> compressed;
  uint fps;
  uint totalCurrentFrameCounter;
  uint width, height;
  uint bitrate;
  FrameStruct *frameOriginal;
  FrameStruct *frameCompressed;
  CodecParamsStruct *paramsStruct;
  struct SwsContext *sws_ctx;
  NvPipe_Codec codec;
  NvPipe_Compression compression;
  NvPipe_Format format;

  LibAvDecoder *fd;

  ImageDecoder id;

  void buildEncoder(YAML::Node _codec_parameters, uint fps);

public:
  NvEncoder(YAML::Node _codec_parameters, uint _fps);

  ~NvEncoder();

  void addFrameStruct(FrameStruct *fs);

  void nextPacket();

  bool hasNextPacket();

  FrameStruct *currentFrameEncoded();

  FrameStruct *currentFrameOriginal();

  CodecParamsStruct *getCodecParamsStruct();

  uint getFps();
};
