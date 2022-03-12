//
// Created by adammpolak on 26-08-2021.
//

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
  int current_frame_counter_ = 0;
  FrameStruct frame_template_;
  std::vector<std::shared_ptr<FrameStruct>> current_frame_;
  int counter_ = 0;
  int nhumans_ = 12; // technically this is a bug as for some reason thingsin the ctor are re-ctored WTF gcc=9.4.0/ubuntu20
  int stopAt_ = 10000000;
  bool has_next_ = true;
public:
  DummyBodyReader(YAML::Node config);
  //DummyBodyReader() { std::cerr << "() constructor" << std::endl << std::flush; }
  //DummyBodyReader(const DummyBodyReader &) { std::cerr << "& copy" << std::endl << std::flush; }
  //DummyBodyReader(DummyBodyReader &&) { std::cerr << "&& copy" << std::endl << std::flush; }

  ~DummyBodyReader();

  /** @brief Get current frame data */
  std::vector<std::shared_ptr<FrameStruct>> GetCurrentFrame();

  /** 
   * @brief Get frame types
   * \return a vector of FrameType, listing available data types 
   */
  std::vector<FrameType> GetType();

  /**
   * @brief Check if there is a next frame
   * \return true if there is a next frame
   */
  bool HasNextFrame();

  /** @brief Go to next frame */
  void NextFrame();

  /** @brief Reset this reader */
  void Reset();

  /** 
   * @brief Go to a given frame
   * \param frame_id target frame number
   */
  virtual void GoToFrame(unsigned int frame_id);

  /**
   * @brief Get current frame number
   * \return current frame number.
   */ 
  unsigned int GetCurrentFrameId();

  /**
   * @brief Get indicative FPS in frame per second.
   * \return the FPS number
   */ 
  unsigned int GetFps();

/*
    void Reset();

  bool HasNextFrame();

  void NextFrame();

  std::vector<std::shared_ptr<FrameStruct>> GetCurrentFrame();

  unsigned int GetCurrentFrameId();

  virtual void GoToFrame(unsigned int frame_id);

  unsigned int GetFps();

  std::vector<FrameType> GetType();
*/
};

} // namespace moetsi::ssp
