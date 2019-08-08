//
// Created by amourao on 07/08/19.
//

#pragma once

#include <ctime>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <thread>
#include <unistd.h>


extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>

#include "../structs/FrameStruct.hpp"

#define MAX_DEPTH_VALUE 4096

void avframeToMat(const AVFrame *frame, cv::Mat &image);

void avframeToMatGray(const AVFrame *frame, cv::Mat &image);

void prepareDecodingStruct(FrameStruct &f, std::unordered_map<std::string, AVCodec *> &pCodecs,
                           std::unordered_map<std::string, AVCodecContext *> &pCodecContexts,
                           std::unordered_map<std::string, AVCodecParameters *> &pCodecParameters);


cv::Mat getFloat(cv::Mat &input);

cv::Mat getUMat(cv::Mat &input);

std::vector<ushort> unique(const cv::Mat &input, bool sort = false);

void prepareGrayDepthFrame(cv::Mat &frame, AVFrame *pFrame);

void prepareDepthFrame(cv::Mat &frame, AVFrame *pFrame);

void prepareColorFrame(cv::Mat &frame, AVFrame *pFrame);

double getPSNR(const cv::Mat &I1, const cv::Mat &I2, int maxValue);

template<typename T>
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
