//
// Created by amourao on 28/08/19.
//

#pragma once

#include <fstream>
#include <iostream>
#include <vector>


extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavformat/avio.h>
#include <libavutil/file.h>
};

#include "../structs/frame_struct.hpp"
#include "libav_types.h"
#include "utils.h"

class ImageDecoder {

private:
  // TODO: convert to new types
  AVFormatContext *av_format_context_;
  AVIOContext *avio_context_;
  AVCodecParameters *av_codec_parameters_;
  AVCodecContext *av_codec_context_;
  AVCodec *codec_;
  AVPacket *packet_;

  CodecParamsStruct *codec_params_struct_;

  uint8_t *avio_ctx_buffer_;
  size_t avio_ctx_buffer_size_ = 4096;

  bool libav_ready_;

  void Init(std::vector<unsigned char> &buffer);

  int DecodePacket(AVFrameSharedP pFrame);
  CodecParamsStruct *GetCodecParamsStruct();

public:
  ImageDecoder();

  ~ImageDecoder();

  void ImageBufferToAVFrame(std::shared_ptr<FrameStruct> &fs,
                            AVFrameSharedP pFrame);
};
