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
  std::shared_ptr<FrameStruct> frame_original_;
  std::shared_ptr<FrameStruct> frame_compressed_;
  unsigned int total_frame_counter_;
  unsigned int fps_;
  unsigned int width_, height_;
  zdepth::DepthCompressor compressor_;
  std::unique_ptr<LibAvDecoder> libav_decoder_;
  ImageDecoder image_decoder_;
  SwsContextSafeP sws_context_;
  std::vector<uint8_t> compressed_buffer_;
  std::shared_ptr<CodecParamsStruct> codec_params_struct_;

  std::string stream_id_;

public:
  ZDepthEncoder(int _fps);

  ~ZDepthEncoder();

  void AddFrameStruct(std::shared_ptr<FrameStruct> &fs);

  void NextPacket();

  bool HasNextPacket();

  std::shared_ptr<FrameStruct> CurrentFrameEncoded();

  std::shared_ptr<FrameStruct> CurrentFrameOriginal();

  std::shared_ptr<CodecParamsStruct> GetCodecParamsStruct();

  unsigned int GetFps();

};


