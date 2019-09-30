//
// Created by amourao on 23-09-2019.
//

#pragma once

#include "idecoder.h"
#include "zdepth.hpp"

class ZDepthDecoder : public IDecoder {
private:
  std::vector<uint16_t> decompressed;
  int width;
  int height;
  zdepth::DepthCompressor decompressor;

public:
  ZDepthDecoder();
  ~ZDepthDecoder();
  void init(std::vector<unsigned char> parameter_data);
  cv::Mat Decode(FrameStruct *frame);
};
