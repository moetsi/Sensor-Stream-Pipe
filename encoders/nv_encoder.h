/**
 * \file nv_encoder.h @brief NvPipe encoder
 */
// Created by amourao on 11-09-2019.
#pragma once

#include <NvPipe.h>
#include <yaml-cpp/yaml.h>

#include "../decoders/libav_decoder.h"
#include "../utils/image_decoder.h"
#include "iencoder.h"
#include "../structs/frame_struct.h"
#include "../utils/utils.h"

#ifdef SSP_WITH_KINECT_SUPPORT
#include "../readers/kinect_reader.h"
#include "../utils/kinect_utils.h"
#endif

namespace moetsi::ssp {

/**
 * @brief NvPipe encoder
 */
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
  /**
   * @brief Constructor
   * \param _codec_parameters Yaml parameters
   * \param _fps Frame per second 
   */
  NvEncoder(YAML::Node _codec_parameters, unsigned int _fps);

  /**
   * @brief Destructor
   */
  ~NvEncoder();

  /** 
   * @brief Add a frame struct
   * \param frame_struct FrameStruct to add
   */
  virtual void AddFrameStruct(std::shared_ptr<FrameStruct> &frame_struct) = 0;

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
