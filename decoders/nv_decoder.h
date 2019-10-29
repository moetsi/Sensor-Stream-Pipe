//
// Created by amourao on 12-09-2019.
//

#pragma once

#include <NvPipe.h>
#include <iostream>
#include <opencv2/core/mat.hpp>

#include "../utils/nvpipe_types.h"
#include "../utils/video_utils.h"
#include "idecoder.h"

class NvDecoder : public IDecoder {
private:
  NvPipe* decoder_;
  std::vector<uint8_t> decompressed_buffer_;
  unsigned int width_;
  unsigned int height_;
  NvPipe_Codec codec_;
  NvPipe_Format format_;

public:
  NvDecoder();
  ~NvDecoder();
  void Init(std::vector<unsigned char> parameter_data);
  cv::Mat Decode(FrameStruct& frame);
};
