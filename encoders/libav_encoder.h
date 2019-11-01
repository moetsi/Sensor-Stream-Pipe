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
#include "../decoders/libav_decoder.h"
#include "../utils/logger.h"

class LibAvEncoder : public IEncoder {
private:
  unsigned int total_frame_counter_;

  AVCodecParametersSafeP av_codec_parameters_;
  AVCodecContextSafeP av_codec_context_;
  AVCodecSafeP av_codec_;

  AVFrameSharedP frame_av_;

  std::shared_ptr<CodecParamsStruct> codec_params_struct_;

  SwsContextSafeP sws_context_;

  YAML::Node codec_parameters_;

  ImageDecoder image_decoder_;

  std::unique_ptr<LibAvDecoder> lib_av_decoder_;

  std::queue<std::shared_ptr<FrameStruct>> buffer_fs_;
  std::queue<AVPacketSharedP> buffer_packet_;

  std::string stream_id_;

  unsigned int fps_;

  bool ready_;

  void Init(std::shared_ptr<FrameStruct> &fs);

  void Encode();

  void EncodeA();

  void PrepareFrame();

  std::vector<unsigned char> CurrentFrameBytes();

public:
  LibAvEncoder(std::string codec_parameters_file, unsigned int fps);

  LibAvEncoder(YAML::Node &_codec_parameters, unsigned int fps);

  ~LibAvEncoder();

  void AddFrameStruct(std::shared_ptr<FrameStruct> &fs);

  void NextPacket();

  bool HasNextPacket();

  std::shared_ptr<FrameStruct> CurrentFrameEncoded();

  std::shared_ptr<FrameStruct> CurrentFrameOriginal();

  std::shared_ptr<CodecParamsStruct> GetCodecParamsStruct();

  unsigned int GetFps();

};
