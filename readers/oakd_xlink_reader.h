//
// Created by amourao on 27-06-2019.
//

#pragma once

#include <atomic>
#include <fstream>
#include <iostream>
#include <vector>

#include "../utils/logger.h"
#include <k4a/k4a.h>

#include <cereal/archives/binary.hpp>

#include "../structs/frame_struct.hpp"
#include "../utils/image_decoder.h"
// #include "../utils/kinect_utils.h"
#include "../utils/video_utils.h"
#include "ireader.h"

extern std::atomic_bool exiting;

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

class OakdXlinkReader : public IReader {
private:
  // std::vector<unsigned int> frame_counter_;

  // bool stream_color_;
  // bool stream_depth_;
  // bool stream_ir_;

  // std::string stream_id_;

  // uint8_t device_index_;
  // k4a_device_configuration_t device_config_;
  // bool record_imu_;
  // int32_t absolute_exposure_value_;
  // k4a_device_t device_;
  // k4a_capture_t capture_;
  // k4a_wait_result_t result_ = K4A_WAIT_RESULT_TIMEOUT;
  // int32_t timeout_ms_;

  // FrameStruct frame_template_;
  // std::vector<std::shared_ptr<CodecParamsStruct>> codec_params_structs_;
  // std::shared_ptr<CameraCalibrationStruct> camera_calibration_struct_;


  // std::vector<std::shared_ptr<FrameStruct>> current_frame_;

public:
  OakdXlinkReader();

  ~OakdXlinkReader();

  void Reset();

  bool HasNextFrame();

  void NextFrame();

  std::vector<std::shared_ptr<FrameStruct>> GetCurrentFrame();

  unsigned int GetCurrentFrameId();

  virtual void GoToFrame(unsigned int frame_id);

  unsigned int GetFps();

  std::vector<unsigned int> GetType();
};
