/**
 * \file iencoder.h @brief IEncoder definition: frame encoder
 */
// Created by amourao on 12-09-2019.

// Extra annotations
/**
 * \file encoders/iencoder.h @brief IEncoder factory
 */
/**
 * \file include/encoders/iencoder.h @brief IEncoder factory
 */

#pragma once

#include "../structs/frame_struct.h"
#include <unordered_map>

namespace moetsi::ssp {

/**
 * @brief IEncoder abstract encoder class 
 */
class IEncoder {

public:
  /** @brief Virtual destructor */
  virtual ~IEncoder() {}

  /** 
   * @brief Add a frame struct
   * \param frame_struct FrameStruct to add
   */
  virtual void AddFrameStruct(std::shared_ptr<FrameStruct> &frame_struct) = 0;

  /**
   * @brief Go to next packet
   */
  virtual void NextPacket() = 0;

  /**
   * @brief Check if there is a next packet
   * \return true if there is a next packet
   */
  virtual bool HasNextPacket() = 0;

  /**
   * @brief Get current encoded frame
   * \return current encoded frame
   */
  virtual std::shared_ptr<FrameStruct> CurrentFrameEncoded() = 0;

  /**
   * @brief Get current frame in its original format 
   * \return current frame in its original format
   */
  virtual std::shared_ptr<FrameStruct> CurrentFrameOriginal() = 0;

  /**
   * @brief Get codec parameters
   * \return codec parameters
   */
  virtual std::shared_ptr<CodecParamsStruct> GetCodecParamsStruct() = 0;

  /**
   * @brief Get FPS
   * \return FPS in frame per second
   */
  virtual unsigned int GetFps() = 0;
};

/**
 * @brief IEncoder factory
 * \param config configuration
 * \param types type to support
 * \return IEncoder instances
 */
std::unordered_map<FrameType, std::shared_ptr<IEncoder>> IEncoderFactory(const std::string & config, const std::vector<FrameType> &types);

}
