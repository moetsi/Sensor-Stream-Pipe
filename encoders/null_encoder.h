/**
 * \file null_encoder.h @breif Straight pipe encoder
 */
// Created by amourao on 16/09/19.
#pragma once

#include "iencoder.h"

namespace moetsi::ssp {

/**
 * @brief Nullencoder Straight pipe encoder
 */
class NullEncoder : public IEncoder {
private:
  std::shared_ptr<FrameStruct> frame;
  unsigned int fps;

public:
  /**
   * @brief Constructor
   * \param _fps frame per seconds
   */
  NullEncoder(int _fps);

  /** @brief Destructor */
  virtual ~NullEncoder();

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
