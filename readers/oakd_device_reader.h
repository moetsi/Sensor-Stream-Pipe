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

#include "human_poses.h"

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

class OakdDeviceReader : public IReader {
private:

  bool stream_rgb = false;
  bool stream_depth = false;
  bool stream_bodies = false;
  uint64_t start_time_in_ms;

  int current_frame_counter_;
  unsigned int fps;
  FrameStruct frame_template_;

  //We use this dictionary to grab pairs of rgb and depth frames that caame from same point in time
  // std::unordered_map<int, std::vector<std::shared_ptr<dai::ImgFrame>>> frames_dictionary;
  struct nn_and_depth { std::shared_ptr<dai::NNData> nn; std::shared_ptr<dai::ImgFrame> depth;};
  std::unordered_map<int, nn_and_depth> frames_dictionary;

  std::vector<std::shared_ptr<FrameStruct>> current_frame_;

  //oakd info
  dai::Pipeline pipeline;
  std::vector<std::string> queueNames;
  std::shared_ptr<dai::node::ColorCamera> camRgb;
  std::shared_ptr<dai::node::MonoCamera> left;
  std::shared_ptr<dai::node::MonoCamera> right;
  std::shared_ptr<dai::node::StereoDepth> stereo;
  std::shared_ptr<dai::node::NeuralNetwork> nn;
  std::shared_ptr<dai::node::XLinkOut> rgbOut;
  std::shared_ptr<dai::node::XLinkOut> depthOut;
  std::shared_ptr<dai::node::XLinkOut> nnXout;

  std::shared_ptr<dai::DataOutputQueue> q;
  std::shared_ptr<dai::DataOutputQueue> qRgb;
  std::shared_ptr<dai::DataOutputQueue> qDepth;
  std::shared_ptr<dai::DataOutputQueue> qNn;
  std::shared_ptr<dai::node::StereoDepth> depth;
  dai::DeviceInfo device_info;
  std::shared_ptr<dai::Device> device;
  dai::CalibrationHandler deviceCalib;
  std::vector<std::vector<float>> cameraIntrinsics;
  float horizontalFocalLengthPixels;
  float verticalFocalLengthPixels;
  float cameraHFOVInRadians;
  
  int stride = 8;
  double input_scale = 256.0 / 720.0;
  float fx = 984.344; // -1;

  //openvino info
  const file_name_t input_model = "../models/human-pose-estimation-3d-0001.xml";
  const std::string input_blob = "@{REL}/models/human-pose-estimation-3d-0001.blob";
  const file_name_t input_image_path= "../models/pointing_middle_of_view.jpg";
  const std::string device_name = "CPU";
  Core ie;
  CNNNetwork network;
  InputInfo::Ptr input_info;
  std::string input_name;
  DataPtr features_output_info;
  DataPtr heatmaps_output_info;
  DataPtr pafs_output_info;
  std::string output_name;
  ExecutableNetwork executable_network;
  InferRequest infer_request;

  std::vector<human_pose_estimation::Pose> previous_poses_2d;
  human_pose_estimation::PoseCommon common;
public:
  OakdDeviceReader(YAML::Node config);

  ~OakdDeviceReader();

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