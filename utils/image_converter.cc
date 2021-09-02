/**
 * \file image_converter.cc @brief Image converter from frame struct to opencv image
 */
// Created by amourao on 28-10-2019.
#include "image_converter.h"
#include "../decoders/libav_decoder.h"
#include "../decoders/zdepth_decoder.h"
#ifdef SSP_WITH_NVPIPE_SUPPORT
#include "../decoders/nv_decoder.h"
#endif

#include <opencv2/imgproc.hpp>

namespace moetsi::ssp {

bool FrameStructToMat(FrameStruct &f, cv::Mat &img,
    std::unordered_map<std::string, std::shared_ptr<IDecoder>> &decoders) {

  bool img_changed = false;

  if (f.frame_data_type == FrameDataType::FrameDataTypeRawRGBA) {         // raw RGBA data
    int rows, cols;
    memcpy(&cols, &f.frame[0], sizeof(int));
    memcpy(&rows, &f.frame[4], sizeof(int));
    img = cv::Mat(rows, cols, CV_8UC4, (void *)&f.frame[8], cv::Mat::AUTO_STEP);
    img_changed = true;
  } else if (f.frame_data_type == FrameDataType::FrameDataTypeGRAY16LE) {  // raw GRAY16LE data
    int rows, cols;
    memcpy(&cols, &f.frame[0], sizeof(int));
    memcpy(&rows, &f.frame[4], sizeof(int));
    img =
        cv::Mat(rows, cols, CV_16UC1, (void *)&f.frame[8], cv::Mat::AUTO_STEP);
    img_changed = true;
  } else if (f.frame_data_type == FrameDataType::FrameDataTypeRaw32FC1) {  // raw 32FC1 data
    int rows, cols;
    memcpy(&cols, &f.frame[0], sizeof(int));
    memcpy(&rows, &f.frame[4], sizeof(int));
    img =
        cv::Mat(rows, cols, CV_32FC1, (void *)&f.frame[8], cv::Mat::AUTO_STEP);
    img_changed = true;   
  } else if (f.frame_data_type == FrameDataType::FrameDataTypeYUV) {  // raw YUV NV12 data in 2 planes
    int rows, cols;
    memcpy(&cols, &f.frame[0], sizeof(int));
    memcpy(&rows, &f.frame[4], sizeof(int));

    cv::Mat y(rows, cols, CV_8UC1, (void *)&f.frame[8], cv::Mat::AUTO_STEP);
    cv::Mat uv(rows/2, cols/2, CV_8UC2, (void *)&f.frame[8 + rows*cols], cv::Mat::AUTO_STEP);
    
    // NV12 (UVUV) vs NV21 (VUVU) : https://www.programmersought.com/article/74944045497/
    cv::cvtColorTwoPlane(y, uv, img, cv::COLOR_YUV2BGR_NV12);
    img_changed = true;
  } else if (f.frame_data_type == FrameDataType::FrameDataTypeU8C1) {  // raw U8C1 data
    int rows, cols;
    memcpy(&cols, &f.frame[0], sizeof(int));
    memcpy(&rows, &f.frame[4], sizeof(int));
    img =
        cv::Mat(rows, cols, CV_8UC1, (void *)&f.frame[8], cv::Mat::AUTO_STEP);
    img_changed = true;
  } else if (f.frame_data_type == FrameDataType::FrameDataTypeImageFrame || f.frame_data_type == FrameDataType::FrameDataTypeLibavPackets) {

    std::string decoder_id = f.stream_id + std::to_string(f.sensor_id);

    if (decoders.find(decoder_id) == decoders.end()) {
      CodecParamsStruct data = f.codec_data;
      // if (data.type == 0) {
      if (data.type == CodecParamsType::CodecParamsTypeAv) {
        std::shared_ptr<LibAvDecoder> fd = std::shared_ptr<LibAvDecoder>(new LibAvDecoder());
        fd->Init(getParams(f));
        decoders[decoder_id] = fd;
      } else if (data.type == CodecParamsType::CodecParamsTypeNvPipe) {
#ifdef SSP_WITH_NVPIPE_SUPPORT
        std::shared_ptr<NvDecoder> fd = std::shared_ptr<NvDecoder>(new NvDecoder());
        fd->Init(data.data);
        decoders[decoder_id] = fd;
#else
        spdlog::error("SSP compiled without \"nvenc\" reader support. Set to "
                      "SSP_WITH_NVPIPE_SUPPORT=ON when configuring with cmake");
        exit(1);
#endif
      } else if (data.type == CodecParamsType::CodecParamsTypeZDepth) {        
        std::shared_ptr<ZDepthDecoder> fd = std::shared_ptr<ZDepthDecoder>(new ZDepthDecoder());
        fd->Init(data.data);
        decoders[decoder_id] = fd;
      }
    }

    std::shared_ptr<IDecoder> decoder = decoders[decoder_id];

    img = decoder->Decode(f);
    img_changed = true;
  }
  return img_changed;
}

} // namespace moetsi::ssp
