//
// Created by amourao on 28/08/19.
//

#pragma once

#include <fstream>
#include <iostream>
#include <vector>


extern "C" {
#ifdef FFMPEG_AS_FRAMEWORK
#include <FFmpeg/avcodec.h>
#include <FFmpeg/avformat.h>
#include <FFmpeg/avio.h>
#include <FFmpeg/file.h>
#else
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavformat/avio.h>
#include <libavutil/file.h>
#endif
};

#include "../structs/frame_struct.hpp"
#include "libav_types.h"
#include "utils.h"

class ImageDecoder {

private:

  AVFormatContextSafeP av_format_context_;
  AVIOContextSafeP avio_context_;
  AVCodecParametersSafePNullDelete av_codec_parameters_;
  AVCodecContextSafeP av_codec_context_;
  AVCodecSafeP codec_;
  AVPacketSharedP packet_;

  std::shared_ptr<CodecParamsStruct> codec_params_struct_;

  unsigned char * avio_ctx_buffer_;
  size_t avio_ctx_buffer_size_ = 4096;

  bool libav_ready_;

  void Init(std::vector<unsigned char> &buffer);

  int DecodePacket(AVFrameSharedP pFrame);


public:
  ImageDecoder();

  ~ImageDecoder();

  void ImageBufferToAVFrame(std::shared_ptr<FrameStruct> &fs,
                            AVFrameSharedP pFrame);
};
