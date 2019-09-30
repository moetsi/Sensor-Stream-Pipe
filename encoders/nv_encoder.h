//
// Created by amourao on 11-09-2019.
//

#pragma once

#include <NvPipe.h>
#include <yaml-cpp/yaml.h>

#include "../decoders/libav_decoder.h"
#include "../utils/image_decoder.h"
#include "iencoder.h"
#include "../readers/kinect_reader.h"
#include "../structs/frame_struct.hpp"
#include "../utils/kinect_utils.h"
#include "../utils/utils.h"


class NvEncoder : public IEncoder {

private:
  NvPipe *encoder;
  std::vector<uint8_t> compressed;
  uint fps;
  uint totalCurrentFrameCounter;
  uint width, height;
  uint64 bitrate;
  FrameStruct *frameOriginal;
  FrameStruct *frameCompressed;
  CodecParamsStruct *paramsStruct;
  struct SwsContext *sws_ctx;
  NvPipe_Codec codec;
  NvPipe_Compression compression;
  NvPipe_Format format;
  std::string stream_id;

  LibAvDecoder *fd;

  ImageDecoder id;

  void buildEncoder(YAML::Node _codec_parameters);

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
