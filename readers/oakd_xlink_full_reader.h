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
#include "classification_results.h"
#include "openvino/openvino.hpp"
#include <iterator>
#include <memory>
#include "common.hpp"
#include "ocv_common.hpp"
#include <string>
#include <vector>

namespace moetsi::ssp { 

#ifndef MAGIC
#define MAGIC 0.84381
#endif

// using namespace InferenceEngine;

class OakdXlinkFullReader : public IReader {
private:

  bool stream_rgb = false;
  bool stream_depth = false;
  bool stream_bodies = false;

  int current_frame_counter_;
  unsigned int fps;
  FrameStruct frame_template_;
  uint64_t start_time;

  //We use this dictionary to grab pairs of rgb and depth frames that caame from same point in time
  struct color_poses_and_depth { std::shared_ptr<dai::ImgFrame> rgb; moetsi::ssp::human_pose_estimation::poses poses; std::shared_ptr<dai::ImgFrame> depth;};
  std::unordered_map<int, color_poses_and_depth> frames_dictionary;
  std::vector<std::shared_ptr<FrameStruct>> current_frame_;

  const std::string device_name = "CPU";

  std::shared_ptr<dai::Pipeline> pipeline;
  std::shared_ptr<dai::node::ColorCamera> camRgb;
  std::shared_ptr<dai::node::MonoCamera> left;
  std::shared_ptr<dai::node::MonoCamera> right;
  std::shared_ptr<dai::node::StereoDepth> stereo;
  std::shared_ptr<dai::node::XLinkIn> controlIn;
  std::shared_ptr<dai::node::XLinkOut> rgbOut;
  std::shared_ptr<dai::node::XLinkOut> depthOut;

  std::shared_ptr<dai::DataInputQueue> controlQueue;
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

  double scale_multiplier1 = MAGIC;
  double scale_multiplier2 = MAGIC;
  double xmagic = MAGIC;
  int sz_x = 256;
  int sz_y = 384;
  bool compute_alt = false;
  double scale_multiplier1_alt = MAGIC;
  double scale_multiplier2_alt = MAGIC;
  double xmagic_alt;
  int sz_x_alt = 256;
  int sz_y_alt = 384;

  struct State {
    //oakd info
    int stride = 8;
    double input_scale = 256.0 / 720.0;
    float fx = 984.344;
    //* int targetX = 256;
    //* int targetY = 384;

    InferenceEngine::Core ie;
    InferenceEngine::CNNNetwork network;
    InferenceEngine::InputInfo::Ptr input_info;
    std::string input_name;
    InferenceEngine::DataPtr features_output_info;
    InferenceEngine::DataPtr heatmaps_output_info;
    InferenceEngine::DataPtr pafs_output_info;
    std::string output_name;
    InferenceEngine::ExecutableNetwork executable_network;
    InferenceEngine::InferRequest infer_request;

    unsigned int rgb_res;
    dai::ColorCameraProperties::SensorResolution rgb_dai_res;    
    unsigned int rgb_dai_preview_y;
    unsigned int rgb_dai_preview_x;
    unsigned int rgb_dai_fps;

    unsigned int depth_res;
    dai::MonoCameraProperties::SensorResolution depth_dai_res;
    unsigned int depth_dai_preview_y;
    unsigned int depth_dai_preview_x;
    unsigned int depth_dai_fps;
    bool depth_dai_sf;
    unsigned int depth_dai_sf_hfr;
    unsigned int depth_dai_sf_num_it;
    unsigned int depth_dai_df;

    std::vector<human_pose_estimation::Pose> previous_poses_2d;
    human_pose_estimation::PoseCommon common;
  };
  std::shared_ptr<State> states[2];
  
  void ResetVino() {
    cameraIntrinsics.clear();
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

  void ResetState(std::shared_ptr<State> &st) {
    if (!!st) {
      auto st2 =  std::make_shared<State>();

      st2->stride = st->stride;
      st2->input_scale = st->input_scale;
      st2->fx = st->fx;
      //st2->targetX = st->targetX;
      //st2->targetY = st->targetY;
      st2->rgb_res = st->rgb_res;
      st2->rgb_dai_res = st->rgb_dai_res;
      st2->rgb_dai_preview_y = st->rgb_dai_preview_y;
      st2->rgb_dai_preview_x = st->rgb_dai_preview_x;
      st2->rgb_dai_fps = st->rgb_dai_fps;
      st2->depth_res = st->depth_res;
      st2->depth_dai_res = st->depth_dai_res;
      st2->depth_dai_preview_y = st->depth_dai_preview_y;
      st2->depth_dai_preview_x = st->depth_dai_preview_x;
      st2->depth_dai_fps = st->depth_dai_fps;
      st2->depth_dai_sf = st->depth_dai_sf;
      st2->depth_dai_sf_hfr = st->depth_dai_sf_hfr;
      st2->depth_dai_sf_num_it = st->depth_dai_sf_num_it;
      st2->depth_dai_df = st->depth_dai_df;
      st = st2;
    } else {
      st = std::make_shared<State>();
    }
  }

  void Init(YAML::Node config, std::shared_ptr<State> &st, int n);
  void SetOrReset();
  void SetOrResetState(const std::shared_ptr<State> &st, int n);

  struct moetsi::ssp::human_pose_estimation::poses getPosesAfterImageResize(int targetX, int targetY, const std::shared_ptr<State> &st, cv::Mat &image, bool isFlex);

  std::string ip_name;
  bool failed = { false };
  std::string model_path;
  std::string model_path2;

public:
  OakdXlinkFullReader(YAML::Node config);

  ~OakdXlinkFullReader();

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
