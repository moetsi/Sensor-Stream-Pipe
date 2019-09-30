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
  NvPipe *encoder_;
  std::vector<uint8_t> compressed_buffer_;
  unsigned int fps_;
  unsigned int total_frame_counter_;
  unsigned int width_, height_;
  uint64 bitrate_;
  FrameStruct *frame_original_;
  FrameStruct *frame_compressed_;
  CodecParamsStruct *codec_params_struct_;
  struct SwsContext *sws_context_;
  NvPipe_Codec codec_;
  NvPipe_Compression compression_;
  NvPipe_Format format_;
  std::string stream_id_;

  LibAvDecoder *lib_av_decoder_;

  ImageDecoder image_decoder_;

  void BuildEncoder(YAML::Node _codec_parameters);

public:
  NvEncoder(YAML::Node _codec_parameters, unsigned int _fps);

  ~NvEncoder();

  void AddFrameStruct(FrameStruct *fs);

  void NextPacket();

  bool HasNextPacket();

  FrameStruct *CurrentFrameEncoded();

  FrameStruct *CurrentFrameOriginal();

  CodecParamsStruct *GetCodecParamsStruct();

  unsigned int GetFps();
};
