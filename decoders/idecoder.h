//
// Created by amourao on 12-09-2019.
//

#pragma once

#include "../structs/frame_struct.hpp"
#include <opencv2/core/mat.hpp>

class IDecoder {

public:
  virtual ~IDecoder() {}

  virtual cv::Mat Decode(FrameStruct& data) = 0;
};