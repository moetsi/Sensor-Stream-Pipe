//
// Created by amourao on 28-10-2019.
//

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

#pragma once

struct AVCodecContextDeleter {
  void operator()(AVCodecContext *ptr) const { avcodec_free_context(&ptr); }
};

struct AVFormatContextDeleter {
  void operator()(AVFormatContext *ptr) const { avformat_free_context(ptr); }
};

struct AVFrameDeleter {
  void operator()(AVFrame *ptr) const { av_frame_free(&ptr); }
};

struct AVPacketDeleter {
  void operator()(AVPacket *ptr) const { av_packet_free(&ptr); }
};

struct AVCodecParametersDeleter {
  void operator()(AVCodecParameters *ptr) const {
    avcodec_parameters_free(&ptr);
  }
};

struct SwsContextDeleter {
  void operator()(SwsContext *ptr) const { sws_freeContext(ptr); }
};

auto AVFrameSharedDeleter = [](AVFrame *ptr) {
  av_frame_free(&ptr);
};

auto AVPacketSharedDeleter = [](AVPacket *ptr) {
  av_packet_free(&ptr);
};

typedef std::unique_ptr<AVFrame, AVFrameDeleter> AVFrameSafeP;
typedef std::shared_ptr<AVFrame> AVFrameSharedP;
typedef std::unique_ptr<AVCodecContext, AVCodecContextDeleter>
    AVCodecContextSafeP;
typedef std::unique_ptr<AVPacket, AVPacketDeleter> AVPacketSafeP;
typedef std::shared_ptr<AVPacket> AVPacketSharedP;
typedef std::unique_ptr<AVCodecParameters, AVCodecParametersDeleter>
    AVCodecParametersSafeP;
typedef std::unique_ptr<struct SwsContext, SwsContextDeleter> SwsContextSafeP;
typedef std::unique_ptr<AVFormatContext, AVFormatContextDeleter>
    AVFormatContextSafeP;
