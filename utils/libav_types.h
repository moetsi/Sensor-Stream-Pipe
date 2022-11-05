/**
 * Namespace libav_types.h @brief Types for libav support
 */
// Created by amourao on 28-10-2019.
#pragma once

extern "C" {
#ifdef FFMPEG_AS_FRAMEWORK
#include <FFmpeg/avcodec.h>
#include <FFmpeg/avformat.h>
#include <FFmpeg/avutil.h>
#include <FFmpeg/imgutils.h>
#include <FFmpeg/opt.h>
#include <FFmpeg/pixdesc.h>
#include <FFmpeg/swscale.h>
#else
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
#endif
}

namespace moetsi::ssp {

struct AVCodecContextDeleter {
  void operator()(AVCodecContext *ptr) const {
    if (ptr)
      avcodec_free_context(&ptr);
  }
};

struct AVFormatContextDeleter {
  void operator()(AVFormatContext *ptr) const {
    if (ptr)
      avformat_free_context(ptr);
  }
};

struct AVFrameDeleter {
  void operator()(AVFrame *ptr) const {
    if (ptr)
      av_frame_free(&ptr);
  }
};

struct AVPacketDeleter {
  void operator()(AVPacket *ptr) const {
    if (ptr)
      av_packet_free(&ptr);
  }
};

struct AVCodecParametersDeleter {
  void operator()(AVCodecParameters *ptr) const {
    if (ptr != nullptr)
      avcodec_parameters_free(&ptr);
  }
};

struct AVCodecParametersNullDeleter {
  void operator()(AVCodecParameters *ptr) const {  }
};

struct SwsContextDeleter {
  void operator()(SwsContext *ptr) const {
    if (ptr)
      sws_freeContext(ptr);
  }
};

struct AVCodecDeleter {
  void operator()(AVCodec *ptr) const {  }
};

struct AVIOContextDeleter {
  void operator()(AVIOContext *ptr) const {
    if (ptr) {
      av_freep(&ptr->buffer);
      av_freep(&ptr);
    }
  }
};

static auto AVFrameSharedDeleter = [](AVFrame *ptr) { av_frame_free(&ptr); };

static auto AVPacketSharedDeleter = [](AVPacket *ptr) { av_packet_free(&ptr); };

typedef std::unique_ptr<AVFrame, AVFrameDeleter> AVFrameSafeP;
typedef std::shared_ptr<AVFrame> AVFrameSharedP;
typedef std::unique_ptr<AVCodecContext, AVCodecContextDeleter>
    AVCodecContextSafeP;
typedef std::unique_ptr<AVPacket, AVPacketDeleter> AVPacketSafeP;
typedef std::shared_ptr<AVPacket> AVPacketSharedP;
typedef std::unique_ptr<AVCodecParameters, AVCodecParametersDeleter>
    AVCodecParametersSafeP;
typedef std::unique_ptr<AVCodecParameters, AVCodecParametersNullDeleter>
    AVCodecParametersSafePNullDelete;
typedef std::unique_ptr<struct SwsContext, SwsContextDeleter> SwsContextSafeP;
typedef std::unique_ptr<AVFormatContext, AVFormatContextDeleter>
    AVFormatContextSafeP;
typedef std::unique_ptr<AVCodec, AVCodecDeleter> AVCodecSafeP;

typedef std::unique_ptr<AVIOContext, AVIOContextDeleter> AVIOContextSafeP;

} // namespace moetsi::ssp
