//
// Created by amourao on 12-09-2019.
//

#pragma once

#include <NvPipe.h>
#include <iostream>
#include <opencv2/core/mat.hpp>

#include "../utils/video_utils.h"
#include "idecoder.h"

class NvDecoder : public IDecoder {
private:
  NvPipe *decoder;
  std::vector<uint8_t> decompressed;
  uint width;
  uint height;
  NvPipe_Codec codec;
  NvPipe_Format format;

public:
  NvDecoder();
  ~NvDecoder();
  void init(std::vector<unsigned char> parameter_data);
  cv::Mat Decode(FrameStruct *frame);
};
