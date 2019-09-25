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

#include "../utils/Logger.h"
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include "../utils/VideoUtils.h"
#include "IDecoder.h"

class LibAvDecoder : public IDecoder {
private:

  AVCodec * pCodec;
  AVCodecContext * pCodecContext;
  AVCodecParameters * pCodecParameter;

public:
  LibAvDecoder();
  ~LibAvDecoder();
  void init(AVCodecParameters *pCodecParameter);
  cv::Mat decode(FrameStruct *data);
};
