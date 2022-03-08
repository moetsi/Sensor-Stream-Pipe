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

class OakdXlinkReader : public IReader {
private:

  bool stream_rgb = false;
  bool stream_depth = false;
  bool stream_bodies = false;

  int current_frame_counter_;
  unsigned int fps;
  FrameStruct frame_template_;
  uint64_t start_time;

  //We use this dictionary to grab pairs of rgb and depth frames that caame from same point in time
  struct color_and_depth { std::shared_ptr<dai::ImgFrame> rgb; std::shared_ptr<dai::ImgFrame> depth;};
  std::unordered_map<int, color_and_depth> frames_dictionary;
  std::vector<std::shared_ptr<FrameStruct>> current_frame_;

  //oakd info
  std::shared_ptr<dai::Pipeline> pipeline;
  std::shared_ptr<dai::node::ColorCamera> camRgb;
  std::shared_ptr<dai::node::MonoCamera> left;
  std::shared_ptr<dai::node::MonoCamera> right;
  std::shared_ptr<dai::node::StereoDepth> stereo;
  std::shared_ptr<dai::node::XLinkOut> rgbOut;
  std::shared_ptr<dai::node::XLinkOut> depthOut;

  std::shared_ptr<dai::DataOutputQueue> q;
  std::shared_ptr<dai::DataOutputQueue> qRgb;
  std::shared_ptr<dai::DataOutputQueue> qDepth;
  std::shared_ptr<dai::node::StereoDepth> depth;
  std::shared_ptr<dai::DeviceInfo> device_info;
  std::shared_ptr<dai::Device> device;
  std::shared_ptr<dai::CalibrationHandler> deviceCalib;
  std::vector<std::vector<float>> cameraIntrinsics;
  float horizontalFocalLengthPixels;
  float verticalFocalLengthPixels;
  float cameraHFOVInRadians;

  int stride = 8;
  double input_scale = 256.0 / 720.0;
  float fx = 984.344;
  
  //openvino info
  const file_name_t input_model = "../models/human-pose-estimation-3d-0001.xml";
  const file_name_t input_image_path= "../models/pointing_middle_of_view.jpg";
  const std::string device_name = "CPU";
  struct State {
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
  };
  std::shared_ptr<State> state;

  void ResetStateAndMisc() {
    cameraIntrinsics.clear();
    state = std::make_shared<State>();

    std::shared_ptr<dai::Pipeline> pipeline_zero;
    pipeline = pipeline_zero;
    std::shared_ptr<dai::node::ColorCamera> camRgb_zero;
    camRgb = camRgb_zero;
    std::shared_ptr<dai::node::MonoCamera> left_zero;
    left = left_zero;
    std::shared_ptr<dai::node::MonoCamera> right_zero;
    right = right_zero;
    std::shared_ptr<dai::node::StereoDepth> stereo_zero;
    stereo = stereo_zero;
    std::shared_ptr<dai::node::XLinkOut> rgbOut_zero;
    rgbOut = rgbOut_zero;
    std::shared_ptr<dai::node::XLinkOut> depthOut_zero;
    depthOut = depthOut_zero;
    std::shared_ptr<dai::DataOutputQueue> q_zero;
    q = q_zero;
    std::shared_ptr<dai::DataOutputQueue> qRgb_zero;
    qRgb = qRgb_zero;
    std::shared_ptr<dai::DataOutputQueue> qDepth_zero;
    qDepth = qDepth_zero;
    std::shared_ptr<dai::node::StereoDepth> depth_zero;
    depth = depth_zero;
    std::shared_ptr<dai::DeviceInfo> device_info_zero;
    device_info = device_info_zero;
    std::shared_ptr<dai::Device> device_zero;
    device = device_zero;
    std::shared_ptr<dai::CalibrationHandler> deviceCalib_zero;
    deviceCalib = deviceCalib_zero;
  }

  std::vector<human_pose_estimation::Pose> previous_poses_2d;
  human_pose_estimation::PoseCommon common;

  void SetOrResetInternals();

  std::string ip_name;
  bool failed = { false };

  unsigned int rgb_res; // = config["rgb_resolution"].as<unsigned int>();
  dai::ColorCameraProperties::SensorResolution rgb_dai_res;    
  unsigned int rgb_dai_preview_y; // = config["rgb_preview_size_y"].as<unsigned int>();
  unsigned int rgb_dai_preview_x; // = config["rgb_preview_size_x"].as<unsigned int>();
  unsigned int rgb_dai_fps; // = config["rgb_fps"].as<unsigned int>();

  unsigned int depth_res; // = config["depth_resolution"].as<unsigned int>();
  dai::MonoCameraProperties::SensorResolution depth_dai_res;
  unsigned int depth_dai_preview_y; //  = config["depth_preview_size_y"].as<unsigned int>();
  unsigned int depth_dai_preview_x; // = config["depth_preview_size_x"].as<unsigned int>();
  unsigned int depth_dai_fps; // = config["depth_fps"].as<unsigned int>();
  bool depth_dai_sf; // = config["depth_spatial_filter"].as<bool>();
  unsigned int depth_dai_sf_hfr; // = config["depth_spatial_hole_filling_radius"].as<unsigned int>();
  unsigned int depth_dai_sf_num_it; // = config["depth_spatial_filter_num_it"].as<unsigned int>();
  unsigned int depth_dai_df; // = config["depth_decimation_factor"].as<unsigned int>();

  std::string model_path;

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
