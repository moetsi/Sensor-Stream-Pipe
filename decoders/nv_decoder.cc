//
// Created by amourao on 12-09-2019.
//

#include "nv_decoder.h"

NvDecoder::NvDecoder() {}

NvDecoder::~NvDecoder() {}

void NvDecoder::Init(std::vector<unsigned char> parameter_data) {

  memcpy(&width_, &parameter_data[0], sizeof(int));
  memcpy(&height_, &parameter_data[4], sizeof(int));

  ushort format__ushort = parameter_data[8];
  ushort codec__ushort = parameter_data[9];

  if (format__ushort == 0) {
    height_ = NVPIPE_RGBA32;
    decompressed_buffer_.resize(width_ * height_ * 4);
  } else if (format__ushort == 1) {
    format_ = NVPIPE_UINT4;
    decompressed_buffer_.resize(width_ * height_ * 0.5);
  } else if (format__ushort == 2) {
    format_ = NVPIPE_UINT8;
    decompressed_buffer_.resize(width_ * height_ * 1);
  } else if (format__ushort == 3) {
    format_ = NVPIPE_UINT16;
    decompressed_buffer_.resize(width_ * height_ * 2);
  } else if (format__ushort == 4) {
    format_ = NVPIPE_UINT32;
    decompressed_buffer_.resize(width_ * height_ * 4);
  }

  if (codec__ushort == 0) {
    codec_ = NVPIPE_H264;
  } else if (codec__ushort == 1) {
    codec_ = NVPIPE_HEVC;
  }

  decoder_ = NvPipe_CreateDecoder(format_, codec_, width_, height_);

  if (decoder_ == nullptr) {
    spdlog::error("Could not create new NVDecoder");
    spdlog::error(NvPipe_GetError(NULL));
    exit(1);
  }
}

cv::Mat NvDecoder::Decode(FrameStruct *data) {
  //TODO: do not crash on failure, wait for I Frame if mid stream
  NvPipe_Decode(decoder_, data->frame.data(), data->frame.size(),
                decompressed_buffer_.data(), width_, height_);

  if (decompressed_buffer_.size() == 0) {
    spdlog::error("Could not decode frame on NvDecoder");
    spdlog::error(NvPipe_GetError(NULL));
    exit(1);
  }

  cv::Mat img;
  if (format_ == NVPIPE_RGBA32)
    img = cv::Mat(height_, width_, CV_8UC4, decompressed_buffer_.data(),
                  cv::Mat::AUTO_STEP);
  else if (format_ == NVPIPE_UINT8)
    img = cv::Mat(height_, width_, CV_8UC1, decompressed_buffer_.data(),
                  cv::Mat::AUTO_STEP);
  else if (format_ == NVPIPE_UINT16)
    img = cv::Mat(height_, width_, CV_16UC1, decompressed_buffer_.data(),
                  cv::Mat::AUTO_STEP);

  return img;
}
