//
// Created by amourao on 23-09-2019.
//

#pragma once

#include "idecoder.h"
#include "zdepth.hpp"

class ZDepthDecoder : public IDecoder {
private:
  std::vector<uint16_t> decompressed_buffer_;
  int width_;
  int height_;
  zdepth::DepthCompressor decompressor_;

public:
  ZDepthDecoder();
  ~ZDepthDecoder();
  void Init(std::vector<unsigned char> parameter_data);
  cv::Mat Decode(FrameStruct *frame);
};
