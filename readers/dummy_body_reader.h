/**
 * \file dummy_body_reader.h @brief Dumy Body Reader
 */
#pragma once

#include <atomic>
#include <fstream>
#include <iostream>
#include <vector>

#include "../utils/logger.h"
#include "../structs/frame_struct.h"
#include "../utils/image_decoder.h"
#include "../structs/body_struct.h"
#include "../utils/video_utils.h"
#include "ireader.h"

namespace moetsi::ssp {

class DummyBodyReader : public IReader {
private:
  int current_frame_counter_;

  FrameStruct frame_template_;

  std::vector<std::shared_ptr<FrameStruct>> current_frame_;

public:
  DummyBodyReader();

  ~DummyBodyReader();

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
