//
// Created by amourao on 12-09-2019.
//

#include "nv_decoder.h"

NvDecoder::NvDecoder() {}

NvDecoder::~NvDecoder() {}

void NvDecoder::init(std::vector<unsigned char> parameter_data) {

  memcpy(&width, &parameter_data[0], sizeof(int));
  memcpy(&height, &parameter_data[4], sizeof(int));

  ushort format_ushort = parameter_data[8];
  ushort codec_ushort = parameter_data[9];

  if (format_ushort == 0) {
    format = NVPIPE_RGBA32;
    decompressed.resize(width * height * 4);
  } else if (format_ushort == 1) {
    format = NVPIPE_UINT4;
    decompressed.resize(width * height * 0.5);
  } else if (format_ushort == 2) {
    format = NVPIPE_UINT8;
    decompressed.resize(width * height * 1);
  } else if (format_ushort == 3) {
    format = NVPIPE_UINT16;
    decompressed.resize(width * height * 2);
  } else if (format_ushort == 4) {
    format = NVPIPE_UINT32;
    decompressed.resize(width * height * 4);
  }

  if (codec_ushort == 0) {
    codec = NVPIPE_H264;
  } else if (codec_ushort == 1) {
    codec = NVPIPE_HEVC;
  }

  decoder = NvPipe_CreateDecoder(format, codec, width, height);

  if (decoder == nullptr) {
    spdlog::error("Could not create new NVDecoder");
    spdlog::error(NvPipe_GetError(NULL));
    exit(1);
  }
}

cv::Mat NvDecoder::Decode(FrameStruct *data) {
  //TODO: do not crash on failure, wait for I Frame if mid stream
  NvPipe_Decode(decoder, data->frame.data(), data->frame.size(),
                decompressed.data(), width, height);

  if (decompressed.size() == 0) {
    spdlog::error("Could not decode frame on NvDecoder");
    spdlog::error(NvPipe_GetError(NULL));
    exit(1);
  }

  cv::Mat img;
  if (format == NVPIPE_RGBA32)
    img = cv::Mat(height, width, CV_8UC4, decompressed.data(),
                  cv::Mat::AUTO_STEP);
  else if (format == NVPIPE_UINT8)
    img = cv::Mat(height, width, CV_8UC1, decompressed.data(),
                  cv::Mat::AUTO_STEP);
  else if (format == NVPIPE_UINT16)
    img = cv::Mat(height, width, CV_16UC1, decompressed.data(),
                  cv::Mat::AUTO_STEP);

  return img;
}
