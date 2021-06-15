//
// Created by amourao on 07/08/19.
//

#pragma once

#include <ctime>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <thread>

#ifdef _WIN32
#include <io.h>
#else
#include <unistd.h>
#endif 

extern "C" {
#ifdef FFMPEG_AS_FRAMEWORK
#include <FFmpeg/avcodec.h>
#include <FFmpeg/avformat.h>
#include <FFmpeg/avutil.h>
#include <FFmpeg/pixdesc.h>
#include <FFmpeg/swscale.h>
#else
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
#endif
}

#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>

#include "../decoders/idecoder.h"
#include "../decoders/zdepth_decoder.h"
#include "../structs/frame_struct.hpp"
#include "../utils/libav_types.h"

#ifdef SSP_WITH_NVPIPE_SUPPORT
#include "../decoders/nv_decoder.h"
#endif

#define MAX_DEPTH_VALUE_16_BITS 65536
#define MAX_DEPTH_VALUE_14_BITS 16384
#define MAX_DEPTH_VALUE_13_BITS 8192
#define MAX_DEPTH_VALUE_12_BITS 4096
#define MAX_DEPTH_VALUE_11_BITS 2048
#define MAX_DEPTH_VALUE_8_BITS 256

void AVFrameToMatYUV(AVFrameSharedP &frame, cv::Mat &image);

void AVFrameToMatGray(AVFrameSharedP &frame, cv::Mat &image);

AVCodecParameters *getParams(FrameStruct& frame_struct);

template <typename T>
void MinMaxFilter(cv::Mat &in_mat, cv::Mat &out_mat, double min, double max) {
  in_mat.copyTo(out_mat);
  for (int i = 0; i < out_mat.rows; i++) {
    for (int j = 0; j < out_mat.cols; j++) {
      if (out_mat.at<T>(i, j) < min) {
        out_mat.at<T>(i, j) = min;
      }
      if (out_mat.at<T>(i, j) > max) {
        out_mat.at<T>(i, j) = max;
      }
    }
  }
}
