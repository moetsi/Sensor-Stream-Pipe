//
// Created by amourao on 11-09-2019.
//

#pragma once

#include <NvPipe.h>
#include <yaml-cpp/yaml.h>

#include "../decoders/libav_decoder.h"
#include "../utils/image_decoder.h"
#include "iencoder.h"
#include "../readers/kinect_reader.h"
#include "../structs/frame_struct.hpp"
#include "../utils/kinect_utils.h"
#include "../utils/utils.h"


class NvEncoder : public IEncoder {

private:
  NvPipeSafeP encoder_;
  std::vector<uint8_t> compressed_buffer_;
  unsigned int fps_;
  unsigned int total_frame_counter_;
  unsigned int width_, height_;
  uint64 bitrate_;
  std::shared_ptr<FrameStruct> frame_original_;
  std::shared_ptr<FrameStruct> frame_compressed_;
  std::shared_ptr<CodecParamsStruct> codec_params_struct_;
  SwsContextSafeP sws_context_;
  NvPipe_Codec codec_;
  NvPipe_Compression compression_;
  NvPipe_Format format_;
  std::string stream_id_;

  std::unique_ptr<LibAvDecoder> lib_av_decoder_;

  ImageDecoder image_decoder_;

  void BuildEncoder(YAML::Node _codec_parameters);

public:
  NvEncoder(YAML::Node _codec_parameters, unsigned int _fps);

  ~NvEncoder();

  void AddFrameStruct(std::shared_ptr<FrameStruct> &fs);

  void NextPacket();

  bool HasNextPacket();

  std::shared_ptr<FrameStruct> CurrentFrameEncoded();

  std::shared_ptr<FrameStruct> CurrentFrameOriginal();

  std::shared_ptr<CodecParamsStruct> GetCodecParamsStruct();

  unsigned int GetFps();
};
