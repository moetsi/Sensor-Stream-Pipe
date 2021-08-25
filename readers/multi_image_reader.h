/**
 * \file multi_image_reader.h @brief Multi image reader 
 */
// Created by amourao on 27-06-2019.
#pragma once

#include <fstream>
#include <iostream>
#include <vector>

// #include <cereal/archives/binary.hpp>

#include "../structs/frame_struct.h"
#include "../utils/image_decoder.h"
#include "image_reader.h"
#include "ireader.h"

namespace moetsi::ssp {

class MultiImageReader: public IReader {
private:
  unsigned int frame_counter_;

  std::vector<std::shared_ptr<IReader>> readers_;
  std::vector<std::shared_ptr<FrameStruct>> current_frame_internal_;

public:
  MultiImageReader(std::vector<std::string> filename);
  ~MultiImageReader();

  /** @brief Get current frame data */
  virtual std::vector<std::shared_ptr<FrameStruct>> GetCurrentFrame();

  /** 
   * @brief Get frame types
   * \return a vector of FrameType, listing available data types 
   */
  virtual std::vector<FrameType> GetType();

  /**
   * @brief Check if there is a next frame
   * \return true if there is a next frame
   */
  virtual bool HasNextFrame();

  /** @brief Go to next frame */
  virtual void NextFrame();

  /** @brief Reset this reader */
  virtual void Reset();

  /** 
   * @brief Go to a given frame
   * \param frame_id target frame number
   */
  virtual void GoToFrame(unsigned int frame_id);

  /**
   * @brief Get current frame number
   * \return current frame number.
   */ 
  virtual unsigned int GetCurrentFrameId();

  /**
   * @brief Get indicative FPS in frame per second.
   * \return the FPS number
   */ 
  virtual unsigned int GetFps();
};

} // namespace moetsi::ssp
