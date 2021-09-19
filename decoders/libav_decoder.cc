/**
 * \file libav_decoder.cc @brief Jpeg/Mpeg decoder
 */ 
// Created by amourao on 12-09-2019.

#include "libav_decoder.h"

namespace moetsi::ssp {

LibAvDecoder::LibAvDecoder() {}

LibAvDecoder::~LibAvDecoder() {}

void LibAvDecoder::Init(AVCodecParameters *codec_parameters) {
  //av_register_all();

  codec_ = std::unique_ptr<AVCodec, AVCodecDeleter>(
      const_cast<AVCodec *>(avcodec_find_decoder(codec_parameters->codec_id)));
  codec_context_ = std::unique_ptr<AVCodecContext, AVCodecContextDeleter>(avcodec_alloc_context3(codec_.get()));
  if (!codec_context_) {
    spdlog::error("Failed to allocated memory for AVCodecContext.");
    exit(1);
  }

  if (avcodec_parameters_to_context(codec_context_.get(), codec_parameters) < 0) {
    spdlog::error("Failed to copy codec params to codec context.");
    exit(1);
  }

  if (avcodec_open2(codec_context_.get(), codec_.get(), NULL) < 0) {
    spdlog::error("Failed to open codec through avcodec_open2.");
    exit(1);
  }
}

cv::Mat LibAvDecoder::Decode(FrameStruct& frame_struct) {
  AVPacketSharedP packet_av =
      std::shared_ptr<AVPacket>(av_packet_alloc(), AVPacketSharedDeleter);
  AVFrameSharedP frame_av =
      std::shared_ptr<AVFrame>(av_frame_alloc(), AVFrameSharedDeleter);

  packet_av->data = frame_struct.frame.data();
  packet_av->size = frame_struct.frame.size();

  cv::Mat img;
  //assert(!!codec_context_);
  //assert(!!packet_av);
  int response = avcodec_send_packet(codec_context_.get(), packet_av.get());
  if (response >= 0) {
    // Return decoded output data (into a frame) from a decoder
    response = avcodec_receive_frame(codec_context_.get(), frame_av.get());
    if (response >= 0) {
      // FrameType::FrameTypeColor = 0
      // FrameType::FrameTypeDepth = 1
      // FrameType::FrameTypeIR = 2
      //if (frame_struct.frame_type == 1 || frame_struct.frame_type == 2) {
      if (frame_struct.frame_type == FrameType::FrameTypeDepth || frame_struct.frame_type == FrameType::FrameTypeIR) {        
        if (codec_context_->pix_fmt == AV_PIX_FMT_GRAY12LE ||
            codec_context_->pix_fmt == AV_PIX_FMT_GRAY16BE) {
          AVFrameToMatGray(frame_av, img);
        } else if (codec_context_->pix_fmt == AV_PIX_FMT_YUV420P) {
          AVFrameToMatYUV(frame_av, img);
          cv::cvtColor(img, img, CV_BGR2GRAY);
          img.convertTo(img, CV_16UC1);
          img *= (MAX_DEPTH_VALUE_12_BITS / MAX_DEPTH_VALUE_8_BITS);
        }
      } else {
        AVFrameToMatYUV(frame_av, img);
      }
    }
  }

  return img;
}

AVFrameSharedP LibAvDecoder::DecodeFrame(FrameStruct &frame_struct) {
  AVPacketSharedP packet_av = std::shared_ptr<AVPacket>(av_packet_alloc(), [](AVPacket *p){ free(p); });
  AVFrameSharedP frame_av =
      std::shared_ptr<AVFrame>(av_frame_alloc(), AVFrameSharedDeleter);

  packet_av->data = frame_struct.frame.data();
  packet_av->size = frame_struct.frame.size();

  cv::Mat img;
  int response = avcodec_send_packet(codec_context_.get(), packet_av.get());
  if (response >= 0) {
    // Return decoded output data (into a frame) from a decoder
    response = avcodec_receive_frame(codec_context_.get(), frame_av.get());
    if (response >= 0) {
    }
  }
  return frame_av;
}

} // namespace moetsi::ssp
