/**
 * \file libav_encoder.h @brief Jpeg/Mpeg encoder 
 */
// Created by amourao on 27-06-2019.
#pragma once

#include <fstream>
#include <iostream>
#include <queue>
#include <vector>
#include <yaml-cpp/yaml.h>

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

#include "../readers/image_reader.h"
#include "../structs/frame_struct.h"
#include "../utils/image_decoder.h"
#include "../utils/video_utils.h"

#include "iencoder.h"
#include "../decoders/libav_decoder.h"
#include "../utils/logger.h"

namespace moetsi::ssp {

/**
 * @brief LibAV encoder for Jpeg/Mpeg
 */
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

  std::shared_ptr<LibAvDecoder> lib_av_decoder_;

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

  /**
   * @brief Constructor
   * \param codec_parameters_file File with codec parameters
   * \param fps Frame per second
   */
  LibAvEncoder(std::string codec_parameters_file, unsigned int fps);

  /**
   * @brief Constructor
   * \param _codec_parameters Yaml codec parameters
   * \param fps Frame per second
   */
  LibAvEncoder(YAML::Node &_codec_parameters, unsigned int fps);

  /**
   * @brief Destructor
   */
  virtual ~LibAvEncoder();

  /** 
   * @brief Add a frame struct
   * \param frame_struct FrameStruct to add
   */
  virtual void AddFrameStruct(std::shared_ptr<FrameStruct> &frame_struct);

  /**
   * @brief Go to next packet
   */
  virtual void NextPacket();

  /**
   * @brief Check if there is a next packet
   * \return true if there is a next packet
   */
  virtual bool HasNextPacket();

  /**
   * @brief Get current encoded frame
   * \return current encoded frame
   */
  virtual std::shared_ptr<FrameStruct> CurrentFrameEncoded();

  /**
   * @brief Get current frame in its original format 
   * \return current frame in its original format
   */
  virtual std::shared_ptr<FrameStruct> CurrentFrameOriginal();

  /**
   * @brief Get codec parameters
   * \return codec parameters
   */
  virtual std::shared_ptr<CodecParamsStruct> GetCodecParamsStruct();

  /**
   * @brief Get FPS
   * \return FPS in frame per second
   */
  virtual unsigned int GetFps();

};

} // namespace moetsi::ssp
