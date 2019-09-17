//
// Created by amourao on 12-09-2019.
//

#include "LibAvDecoder.h"
#include <opencv2/imgproc.hpp>

LibAvDecoder::LibAvDecoder() {}

LibAvDecoder::~LibAvDecoder() {}

void LibAvDecoder::init(AVCodecParameters *pCodecParameter) {
  av_register_all();

  pCodec = avcodec_find_decoder(pCodecParameter->codec_id);
  pCodecContext = avcodec_alloc_context3(pCodec);
  if (!pCodecContext) {
    std::cerr << "failed to allocated memory for AVCodecContext" << std::endl;
    exit(1);
  }

  if (avcodec_parameters_to_context(pCodecContext, pCodecParameter) < 0) {
    std::cerr << ("failed to copy codec params to codec context") << std::endl;
    exit(1);
  }

  if (avcodec_open2(pCodecContext, pCodec, NULL) < 0) {
    std::cerr << ("failed to open codec through avcodec_open2") << std::endl;
    exit(1);
  }
}

cv::Mat LibAvDecoder::decode(FrameStruct *frame) {
  AVPacket *pPacket = av_packet_alloc();
  AVFrame *pFrame = av_frame_alloc();

  pPacket->data = frame->frame.data();
  pPacket->size = frame->frame.size();

  cv::Mat img;
  int response = avcodec_send_packet(pCodecContext, pPacket);
  if (response >= 0) {
    // Return decoded output data (into a frame) from a decoder
    response = avcodec_receive_frame(pCodecContext, pFrame);
    if (response >= 0) {
      if (frame->frameType == 1 || frame->frameType == 2) {
        if (pCodecContext->pix_fmt == AV_PIX_FMT_GRAY12LE ||
            pCodecContext->pix_fmt == AV_PIX_FMT_GRAY16BE) {
          avframeToMatGray(pFrame, img);
        } else if (pCodecContext->pix_fmt == AV_PIX_FMT_YUV420P) {
          avframeToMatYUV(pFrame, img);
          cv::cvtColor(img, img, CV_BGR2GRAY);
          img.convertTo(img, CV_16UC1);
          img *= (MAX_DEPTH_VALUE_12_BITS / MAX_DEPTH_VALUE_8_BITS);
        }
      } else {
        avframeToMatYUV(pFrame, img);
      }
    }
  }
  av_frame_free(&pFrame);
  av_packet_free(&pPacket);
  return img;
}
