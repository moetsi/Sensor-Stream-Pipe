//
// Created by amourao on 23-09-2019.
//

#include "zdepth_decoder.h"
ZDepthDecoder::ZDepthDecoder() {}

ZDepthDecoder::~ZDepthDecoder() {}

void ZDepthDecoder::Init(std::vector<unsigned char> parameter_data) {
  memcpy(&width_, &parameter_data[0], sizeof(int));
  memcpy(&height_, &parameter_data[4], sizeof(int));
}

cv::Mat ZDepthDecoder::Decode(FrameStruct& frame) {

  zdepth::DepthResult result =
      decompressor_.Decompress(frame.frame, width_, height_, decompressed_buffer_);
  if (result != zdepth::DepthResult::Success) {
    // Handle input error
  }

  return cv::Mat(height_, width_, CV_16UC1, decompressed_buffer_.data(),
                 cv::Mat::AUTO_STEP);
  ;
}
