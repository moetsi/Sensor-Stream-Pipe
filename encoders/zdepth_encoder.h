/**
 * \file zdepth_encoder.h ZDepth @brief encoder
 */
// Created by amourao on 23-09-2019.
#pragma once

#include "zdepth.hpp"
#include <yaml-cpp/yaml.h>
#include <opencv2/imgproc.hpp>

#include "iencoder.h"
#include "../decoders/libav_decoder.h"
#include "../utils/image_decoder.h"

namespace moetsi::ssp {

/**
 * @brief ZDepth encoder 
 */
class ZDepthEncoder: public IEncoder {
private:
  std::shared_ptr<FrameStruct> frame_original_;
  std::shared_ptr<FrameStruct> frame_compressed_;
  unsigned int total_frame_counter_;
  unsigned int send_I_frame_interval_;
  unsigned int fps_;
  unsigned int width_, height_;
  zdepth::DepthCompressor compressor_;
  std::unique_ptr<LibAvDecoder> libav_decoder_;
  ImageDecoder image_decoder_;
  SwsContextSafeP sws_context_;
  std::vector<uint8_t> compressed_buffer_;
  std::shared_ptr<CodecParamsStruct> codec_params_struct_;

  std::string stream_id_;

public:
  /**
   * @brief Constructor
   * \param _codec_parameters
   * \param _fps Frame per second
   */
  ZDepthEncoder(YAML::Node& _codec_parameters, int _fps);

  /**
   * @brief Destructor
   */
  ~ZDepthEncoder();

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
