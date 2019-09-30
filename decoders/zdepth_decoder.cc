//
// Created by amourao on 23-09-2019.
//

#include "zdepth_decoder.h"
ZDepthDecoder::ZDepthDecoder() {}

ZDepthDecoder::~ZDepthDecoder() {}

void ZDepthDecoder::init(std::vector<unsigned char> parameter_data) {
  memcpy(&width, &parameter_data[0], sizeof(int));
  memcpy(&height, &parameter_data[4], sizeof(int));
}

cv::Mat ZDepthDecoder::Decode(FrameStruct *frame) {

  //TODO: do not crash on failure, wait for I Frame if mid stream
  zdepth::DepthResult result =
      decompressor.Decompress(frame->frame, width, height, decompressed);
  if (result != zdepth::DepthResult::Success) {
    // Handle input error
  }

  return cv::Mat(height, width, CV_16UC1, decompressed.data(),
                 cv::Mat::AUTO_STEP);
  ;
}
