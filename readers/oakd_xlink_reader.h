//
// Created by adammpolak on 26-09-2021.
//

#pragma once

#include <atomic>
#include <fstream>
#include <iostream>
#include <vector>

#include "../utils/logger.h"

#include <cereal/archives/binary.hpp>

#include "../structs/frame_struct.h"
#include "../utils/image_decoder.h"
#include "../utils/video_utils.h"
#include "ireader.h"

//Header for making dummy bodies
#include "dummy_body_reader.h"

//Depth AI header
#include "depthai/depthai.hpp"
//Done Depth AI header

// OPENVINO HEADERS
/**
 * @brief Define names based depends on Unicode path support
 */
#if defined(ENABLE_UNICODE_PATH_SUPPORT) && defined(_WIN32)
    #define tcout                  std::wcout
    #define file_name_t            std::wstring
    #define imread_t               imreadW
    #define ClassificationResult_t ClassificationResultW
#else
    #define tcout                  std::cout
    #define file_name_t            std::string
    #define imread_t               cv::imread
    #define ClassificationResult_t ClassificationResult
#endif
#include <samples/classification_results.h>
#include <inference_engine.hpp>
#include <iterator>
#include <memory>
#include <samples/common.hpp>
#include <samples/ocv_common.hpp>
#include <string>
#include <vector>

namespace moetsi::ssp { 

using namespace InferenceEngine;
//DONE OPENVINO HEADERS

class OakdXlinkReader : public IReader {
private:
  int current_frame_counter_;

  FrameStruct frame_template_;

  std::vector<std::shared_ptr<FrameStruct>> current_frame_;

  //oakd info
  dai::Pipeline pipeline;
  std::shared_ptr<dai::DataOutputQueue> q;
  std::shared_ptr<dai::DataOutputQueue> qRgb;
  std::shared_ptr<dai::node::StereoDepth> depth;
  std::shared_ptr<dai::node::ColorCamera> camRgb;
  std::shared_ptr<dai::node::XLinkOut> xoutRgb;
  dai::DeviceInfo device_info;
  std::shared_ptr<dai::Device> device;

  //openvino info
  const file_name_t input_model = "../models/human-pose-estimation-3d.xml";
  const file_name_t input_image_path= "dummy_value.png";
  const std::string device_name = "CPU";
  Core ie;
  CNNNetwork network;
  InputInfo::Ptr input_info;
  std::string input_name;
  DataPtr output_info;
  std::string output_name;
  ExecutableNetwork executable_network;

public:
  OakdXlinkReader(YAML::Node config);

  ~OakdXlinkReader();

  void Reset();

  bool HasNextFrame();

  void NextFrame();

  std::vector<std::shared_ptr<FrameStruct>> GetCurrentFrame();

  unsigned int GetCurrentFrameId();

  virtual void GoToFrame(unsigned int frame_id);

  unsigned int GetFps();

  std::vector<FrameType> GetType();
};

} // namespace