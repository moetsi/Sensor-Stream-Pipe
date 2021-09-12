/**
 * \file ireader.h @brief Reader interface to SSP
 */
// Created by amourao on 14-08-2019.
#pragma once

#include "../structs/frame_struct.h"

namespace moetsi::ssp {

/**
 * @brief Setup logging
 * \param level logging level
 * \param file logging file
 */
void SetupLogging(const std::string &level, const std::string &file);

/**
 * @brief SSP reader interface - abstract class.
 */ 
class IReader {

public:
  /** @brief Destructor */
  virtual ~IReader() {}

  /** @brief Get current frame data */
  virtual std::vector<std::shared_ptr<FrameStruct>> GetCurrentFrame() = 0;

#ifndef SWIG
  /** 
   * @brief Get frame types
   * \return a vector of FrameType, listing available data types 
   */
  virtual std::vector<FrameType> GetType() = 0;
#endif

  std::vector<FrameType> GetFrameType() { return GetType(); }

  /**
   * @brief Check if there is a next frame
   * \return true if there is a next frame
   */
  virtual bool HasNextFrame() = 0;

  /** @brief Go to next frame */
  virtual void NextFrame() = 0;

  /** @brief Reset this reader */
  virtual void Reset() = 0;

  /** 
   * @brief Go to a given frame
   * \param frame_id target frame number
   */
  virtual void GoToFrame(unsigned int frame_id) = 0;

  /**
   * @brief Get current frame number
   * \return current frame number.
   */ 
  virtual unsigned int GetCurrentFrameId() = 0;

  /**
   * @brief Get indicative FPS in frame per second.
   * \return the FPS number
   */ 
  virtual unsigned int GetFps() = 0;
};

/**
 * @brief IReader factory
 * \param config configuration
 * \return an IReader instance
 */
std::shared_ptr<IReader> IReaderFactory(const std::string & config);

} // namespace moetsi::ssp