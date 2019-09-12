//
// Created by amourao on 12-09-2019.
//

#include "FrameDecoder.h"

FrameDecoder::FrameDecoder() {}

FrameDecoder::~FrameDecoder() {}

void FrameDecoder::init(AVCodecParameters *pCodecParameter) {
  av_register_all();

  pFrame = av_frame_alloc();

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

cv::Mat FrameDecoder::decode(std::vector<unsigned char> *data) {
  AVPacket *pPacket = av_packet_alloc();

  pPacket->data = data->data();
  pPacket->size = data->size();

  cv::Mat img;
  int response = avcodec_send_packet(pCodecContext, pPacket);
  if (response >= 0) {
    // Return decoded output data (into a frame) from a decoder
    response = avcodec_receive_frame(pCodecContext, pFrame);
    if (response >= 0) {
      if (pCodecContext->pix_fmt == AV_PIX_FMT_GRAY12LE ||
          pCodecContext->pix_fmt == AV_PIX_FMT_GRAY16BE) {
        avframeToMatGray(pFrame, img);
      } else {
        avframeToMatYUV(pFrame, img);
      }
    }
  }

  av_packet_free(&pPacket);
  return img;
}
