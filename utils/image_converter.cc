//
// Created by amourao on 28-10-2019.
//

#include "image_converter.h"

bool FrameStructToMat(FrameStruct &f, cv::Mat &img,
                      std::unordered_map<std::string, IDecoder *> &decoders) {
  std::string decoder_id = f.stream_id + std::to_string(f.sensor_id);

  if (decoders.find(decoder_id) == decoders.end()) {
    CodecParamsStruct data = f.codec_data;
    if (data.type == 0) {
      LibAvDecoder *fd = new LibAvDecoder();
      fd->Init(getParams(f));
      decoders[decoder_id] = fd;
    } else if (data.type == 1) {
#ifdef SSP_WITH_NVPIPE_SUPPORT
      NvDecoder *fd = new NvDecoder();
      fd->Init(data.data);
      decoders[decoder_id] = fd;
#else
      spdlog::error("SSP compiled without \"nvenc\" reader support. Set to "
                    "SSP_WITH_NVPIPE_SUPPORT=ON when configuring with cmake");
      exit(1);
#endif
    } else if (data.type == 2) {
      ZDepthDecoder *fd = new ZDepthDecoder();
      fd->Init(data.data);
      decoders[decoder_id] = fd;
    }
  }

  bool img_changed = false;

  if (f.frame_data_type == 2) {
    int rows, cols;
    memcpy(&cols, &f.frame[0], sizeof(int));
    memcpy(&rows, &f.frame[4], sizeof(int));
    img = cv::Mat(rows, cols, CV_8UC4, (void *)&f.frame[8], cv::Mat::AUTO_STEP);
    img_changed = true;
  } else if (f.frame_data_type == 3) {
    int rows, cols;
    memcpy(&cols, &f.frame[0], sizeof(int));
    memcpy(&rows, &f.frame[4], sizeof(int));
    img =
        cv::Mat(rows, cols, CV_16UC1, (void *)&f.frame[8], cv::Mat::AUTO_STEP);
    img_changed = true;
  } else if (f.frame_data_type == 0 || f.frame_data_type == 1) {

    IDecoder *decoder;

    if (decoders.find(decoder_id) == decoders.end()) {
      CodecParamsStruct data = f.codec_data;
      if (data.type == 0) {
        LibAvDecoder *fd = new LibAvDecoder();
        fd->Init(getParams(f));
        decoders[decoder_id] = fd;
#ifdef SSP_WITH_NVPIPE_SUPPORT
      } else if (data.type == 1) {
        NvDecoder *fd = new NvDecoder();
        fd->Init(data.data);
        decoders[decoder_id] = fd;
#endif
      }
    }

    decoder = decoders[decoder_id];

    img = decoder->Decode(f);
    img_changed = true;
  }
  return img_changed;
}
