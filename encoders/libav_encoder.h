//
// Created by amourao on 27-06-2019.
//

#pragma once

#include <fstream>
#include <iostream>
#include <queue>
#include <vector>

#include <cereal/archives/binary.hpp>

#include <yaml-cpp/yaml.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

#include "../readers/image_reader.h"
#include "../structs/frame_struct.hpp"
#include "../utils/image_decoder.h"
#include "../utils/video_utils.h"

#include "iencoder.h"

class LibAvEncoder : public IEncoder {
private:
  unsigned int total_frame_counter_;

  AVCodecParameters *av_codec_parameters_;
  AVCodecContext *av_codec_context_;
  AVCodec *av_codec_;

  AVFrame *frame_av_;
  AVPacket *packet_av_;

  CodecParamsStruct *codec_params_struct_;

  struct SwsContext *sws_context_;

  YAML::Node codec_parameters_;

  ImageDecoder image_decoder_;

  std::queue<FrameStruct *> buffer_fs_;
  std::queue<AVPacket *> buffer_packet_;

  std::string stream_id_;

  unsigned int fps_;

  bool ready_;

  void Init(FrameStruct *fs);

  void Encode();

  void EncodeA(AVCodecContext *enc_ctx, AVFrame *frame, AVPacket *pkt);

  void PrepareFrame();

  std::vector<unsigned char> CurrentFrameBytes();

public:
  LibAvEncoder(std::string codec_parameters_file, unsigned int fps);

  LibAvEncoder(YAML::Node &_codec_parameters, unsigned int fps);

  ~LibAvEncoder();

  void AddFrameStruct(FrameStruct *fs);

  void NextPacket();

  bool HasNextPacket();

  FrameStruct *CurrentFrameEncoded();

  FrameStruct *CurrentFrameOriginal();

  CodecParamsStruct *GetCodecParamsStruct();

  unsigned int GetFps();

};
