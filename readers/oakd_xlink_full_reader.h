//
// Created by adammpolak on 26-09-2021.
//

#pragma once

#include <atomic>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <map>
#include <optional>
#include <any>
#include <variant>
#include <cassert>
#include <memory>
#include <stdexcept>

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
#include "args_helper.hpp"

namespace moetsi::ssp { 

// using namespace InferenceEngine;

class TwoStageHostSeqSync {
public:
    void add_msg(const std::shared_ptr<void>& msg, const std::string& name);
    std::unordered_map<std::string, std::shared_ptr<void>> get_msgs();

private:
    std::unordered_map<std::string, std::unordered_map<std::string, std::shared_ptr<void>>> msgs;
};

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
  std::shared_ptr<dai::node::XLinkOut> rgbOut;

  std::shared_ptr<dai::node::ImageManip> person_det_manip;

  std::shared_ptr<dai::node::MobileNetDetectionNetwork> person_nn;
  std::shared_ptr<dai::node::XLinkOut> person_det_xout;
  std::shared_ptr<dai::node::Script> image_manip_script;

  std::shared_ptr<dai::node::ImageManip> recognition_manip;
  std::shared_ptr<dai::node::NeuralNetwork> recognition_nn;
  std::shared_ptr<dai::node::XLinkOut> recognition_nn_xout;
  
  std::shared_ptr<dai::node::MonoCamera> left;
  std::shared_ptr<dai::node::MonoCamera> right;
  std::shared_ptr<dai::node::StereoDepth> stereo;
  std::shared_ptr<dai::node::XLinkOut> depthOut;

  std::shared_ptr<dai::DeviceInfo> device_info;
  std::shared_ptr<dai::Device> device;

  std::unordered_map<std::string, std::shared_ptr<dai::DataOutputQueue>> queues;

  TwoStageHostSeqSync sync;
  std::vector<std::vector<float>> results;

  struct State {
    //oakd info
    ov::Core ie2;
    std::shared_ptr<ov::Model> model;
    ov::CompiledModel compiled_model;

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
    std::shared_ptr<dai::DeviceInfo> device_info_zero;
    device_info = device_info_zero;
    std::shared_ptr<dai::Device> device_zero;
    device = device_zero;

  }

  void ResetState(std::shared_ptr<State> &st) {
    if (!!st) {
      auto st2 =  std::make_shared<State>();

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

  std::string ip_name;
  bool failed = { false };
  std::string model_detection_path;
  std::string model_detection_path2;
  std::string model_reid_path;
  std::string model_reid_path2;

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
