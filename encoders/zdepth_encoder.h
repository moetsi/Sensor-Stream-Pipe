//
// Created by amourao on 23-09-2019.
//

#pragma once

#include "zdepth.hpp"
#include <yaml-cpp/yaml.h>
#include <opencv2/imgproc.hpp>

#include "iencoder.h"
#include "../decoders/libav_decoder.h"
#include "../utils/image_decoder.h"


class ZDepthEncoder: public IEncoder {
private:
  FrameStruct *frame_original_;
  FrameStruct *frame_compressed_;
  unsigned int total_frame_counter_;
  unsigned int fps_;
  unsigned int width_, height_;
  zdepth::DepthCompressor compressor_;
  LibAvDecoder *libav_decoder_;
  ImageDecoder image_decoder_;
  struct SwsContext *sws_context_;
  std::vector<uint8_t> compressed_buffer_;
  CodecParamsStruct *codec_params_struct_;

  std::string stream_id_;

public:
  ZDepthEncoder(int _fps);

  ~ZDepthEncoder();

  void AddFrameStruct(FrameStruct *fs);

  void NextPacket();

  bool HasNextPacket();

  FrameStruct *CurrentFrameEncoded();

  FrameStruct *CurrentFrameOriginal();

  CodecParamsStruct *GetCodecParamsStruct();

  unsigned int GetFps();

};


