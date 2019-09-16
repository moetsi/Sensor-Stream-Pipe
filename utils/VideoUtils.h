//
// Created by amourao on 07/08/19.
//

#pragma once

#include <ctime>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <thread>
#include <unistd.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>

#include "../structs/FrameStruct.hpp"

#define MAX_DEPTH_VALUE_16_BITS 65536
#define MAX_DEPTH_VALUE_14_BITS 16384
#define MAX_DEPTH_VALUE_13_BITS 8192
#define MAX_DEPTH_VALUE_12_BITS 4096
#define MAX_DEPTH_VALUE_11_BITS 2048
#define MAX_DEPTH_VALUE_8_BITS 256

void avframeToMatYUV(const AVFrame *frame, cv::Mat &image);

void avframeToMatGray(const AVFrame *frame, cv::Mat &image);

void prepareDecodingStruct(
        FrameStruct *f, std::unordered_map<std::string, AVCodec *> &pCodecs,
        std::unordered_map<std::string, AVCodecContext *> &pCodecContexts,
        std::unordered_map<std::string, AVCodecParameters *> &pCodecParameters);

template <typename T>
void minMaxFilter(cv::Mat &inmat, cv::Mat &outmat, double min, double max) {
  inmat.copyTo(outmat);
  for (uint i = 0; i < outmat.rows; i++) {
    for (uint j = 0; j < outmat.cols; j++) {
      if (outmat.at<T>(i, j) < min) {
        outmat.at<T>(i, j) = min;
      }
      if (outmat.at<T>(i, j) > max) {
        outmat.at<T>(i, j) = max;
      }
    }
  }
}
