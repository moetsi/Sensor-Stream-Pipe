/**
 * \file kinect_reader.h @brief Kinect driver
 */
// Created by amourao on 27-06-2019.
#pragma once

#include <atomic>
#include <fstream>
#include <iostream>
#include <vector>

#include "../utils/logger.h"
#include <k4a/k4a.h>

#include <cereal/archives/binary.hpp>

#include "../structs/frame_struct.h"
#include "../utils/image_decoder.h"
#include "../utils/kinect_utils.h"
#include "../utils/video_utils.h"
#include "ireader.h"

namespace moetsi::ssp {

extern std::atomic_bool exiting;

inline static uint32_t k4a_convert_fps_to_uint(k4a_fps_t fps) {
  uint32_t fps_int;
  switch (fps) {
  case K4A_FRAMES_PER_SECOND_5:
    fps_int = 5;
    break;
  case K4A_FRAMES_PER_SECOND_15:
    fps_int = 15;
    break;
  case K4A_FRAMES_PER_SECOND_30:
    fps_int = 30;
    break;
  default:
    fps_int = 0;
    break;
  }
  return fps_int;
}

// call k4a_device_close on every failed CHECK
#define CHECK(x, device)                                                       \
  {                                                                            \
    auto retval = (x);                                                         \
    if (retval) {                                                              \
      spdlog::error("\"Runtime error: {} returned {} ", #x, retval);           \
      k4a_device_close(device);                                                \
      exit(1);                                                                 \
    }                                                                          \
  }

class KinectReader : public IReader {
private:
  std::vector<unsigned int> frame_counter_;

  bool stream_color_;
  bool stream_depth_;
  bool stream_ir_;

  std::string stream_id_;

  uint8_t device_index_;
  k4a_device_configuration_t device_config_;
  bool record_imu_;
  int32_t absolute_exposure_value_;
  k4a_device_t device_;
  k4a_capture_t capture_;
  k4a_wait_result_t result_ = K4A_WAIT_RESULT_TIMEOUT;
  int32_t timeout_ms_;

  FrameStruct frame_template_;
  std::vector<std::shared_ptr<CodecParamsStruct>> codec_params_structs_;
  std::shared_ptr<CameraCalibrationStruct> camera_calibration_struct_;

  std::vector<std::shared_ptr<FrameStruct>> current_frame_;

public:
  KinectReader(uint8_t device_index, ExtendedAzureConfig device_config);

  ~KinectReader();

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
  virtual void NextFrame(const std::vector<std::string> frame_types_to_pull = {});

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
