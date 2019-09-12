//
// Created by amourao on 12-09-2019.
//

#pragma once

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

#include <unordered_map>


#include <iostream>
#include <opencv2/core/mat.hpp>

#include "../utils/VideoUtils.h"
#include "IDecoder.h"

class FrameDecoder: public IDecoder {
private:

  AVCodec * pCodec;
  AVCodecContext * pCodecContext;
  AVCodecParameters * pCodecParameter;

  AVFrame* pFrame;

public:

  FrameDecoder();
  ~FrameDecoder();
  void init(AVCodecParameters *pCodecParameter);
  cv::Mat decode(std::vector<unsigned char>* data);



};
