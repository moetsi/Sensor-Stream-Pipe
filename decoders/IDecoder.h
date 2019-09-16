//
// Created by amourao on 12-09-2019.
//

#pragma once

#include "../structs/FrameStruct.hpp"
#include <opencv2/core/mat.hpp>

class IDecoder {

public:
  virtual cv::Mat decode(FrameStruct *data) = 0;
};