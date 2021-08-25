/**
 * \file image_reader.h @brief Image reader
 */
// Created by amourao on 27-06-2019.
#pragma once

#include <fstream>
#include <iostream>
#include <vector>

#include "../structs/frame_struct.h"
#include "../utils/image_decoder.h"
#include "ireader.h"

namespace moetsi::ssp {

class ImageReader : public IReader {
private:
  unsigned int frame_counter_;
  unsigned int fps_;
  std::string scene_desc_;
  unsigned int sensor_id_;
  unsigned int device_id_;
  FrameType frame_type_;
  std::string stream_id_;

  std::shared_ptr<CodecParamsStruct> codec_params_struct_;

  std::shared_ptr<FrameStruct> current_frame_internal_;

  std::vector<std::string> frame_lines_;

  std::vector<unsigned char> ReadFile(std::string &filename);

  std::shared_ptr<FrameStruct> CreateFrameStruct(unsigned int frame_id);

public:
  ImageReader(std::string filename);
  virtual ~ImageReader();

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
