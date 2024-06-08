//
// Created by adammpolak on 26-09-2021.
//

#pragma once

#include <atomic>
#include <fstream>
#include <iostream>
#include <sstream>
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
#include "nlohmann/json.hpp"
#include "../utils/logger.h"
#include <algorithm>

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

// header files needed for the tracker
#include "tracker.hpp"
#include "descriptor.hpp"
#include "distance.hpp"
#include "utils/ocv_common.hpp"
// #include "monitor/presenter.h"

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
#include "utils/common.hpp"
#include "utils/ocv_common.hpp"
#include <string>
#include <vector>
#include "utils/args_helper.hpp"
#include "fp16/fp16.h"

namespace moetsi::ssp { 

// using namespace InferenceEngine;
struct MessageData {
    std::shared_ptr<dai::ImgFrame> color;
    std::shared_ptr<dai::ImgFrame> depth;
    std::shared_ptr<dai::SpatialImgDetections> person_detection;
    std::shared_ptr<void> face_detection;
    std::vector<std::shared_ptr<dai::NNData>> recognitions;
    std::vector<std::shared_ptr<cv::Mat>> fast_descriptions;
};

class TwoStageHostSeqSync {
public:
    void add_msg(const std::shared_ptr<void>& msg, const std::string& name);
    MessageData get_msgs(const std::string& sync_mode);

private:
    std::unordered_map<int64_t, MessageData> msgs;
};

class OakdXlinkFullReader : public IReader {
private:

  std::string client_key_;
  std::string environment_name_;
  std::string sensor_name_;

  //This is used to set whether to pull RGB/Depth frames from the oakd device to the host for debug visualization
  bool send_rgb_to_host_and_visualize = false;
  bool send_depth_to_host_and_visualize = false;

  bool stream_rgb = false;
  bool stream_depth = false;
  bool stream_bodies = false;

  // Used for node fps debugging
  int current_rgb_frame_counter_;
  int current_detection_frame_counter_;
  int current_recognition_frame_counter_;

  int current_frame_counter_;
  unsigned int fps;
  FrameStruct frame_template_;
  uint64_t start_time;

  unsigned int rgb_dai_preview_y;
  unsigned int rgb_dai_preview_x;

  std::vector<std::shared_ptr<FrameStruct>> current_frame_;

  std::shared_ptr<dai::Pipeline> pipeline;
  std::shared_ptr<dai::node::ColorCamera> camRgb;
  std::shared_ptr<dai::node::MonoCamera> left;
  std::shared_ptr<dai::node::MonoCamera> right;
  std::shared_ptr<dai::node::StereoDepth> stereo;

  // PERSON DETECTION
  unsigned int rgb_person_det_nn_in_x_res = 544;
  unsigned int rgb_person_det_nn_in_y_res = 320;
  std::shared_ptr<dai::node::ImageManip> person_det_manip;
  std::shared_ptr<dai::node::MobileNetSpatialDetectionNetwork> person_nn;
  std::shared_ptr<dai::node::XLinkOut> person_det_xout;

  // RGB AND DEPTH OUTPUT
  std::shared_ptr<dai::node::Script> rgb_and_depth_out_script;
  std::shared_ptr<dai::node::XLinkOut> rgbOut;
  std::shared_ptr<dai::node::XLinkOut> depthOut;
  std::shared_ptr<dai::node::XLinkIn> control_xlinkin;
  std::shared_ptr<dai::node::Script> control_script;

  // Strong Recognition
  cv::Size strong_descriptor_size = cv::Size(256,1);
  unsigned int rgb_person_reid_strong_nn_in_x_res = 128;
  unsigned int rgb_person_reid_strong_nn_in_y_res = 256;
  std::shared_ptr<dai::node::Script> person_config_manip_for_reid_nn_script;
  std::shared_ptr<dai::node::ImageManip> recognition_manip;
  std::shared_ptr<dai::node::NeuralNetwork> recognition_nn;
  std::shared_ptr<dai::node::XLinkOut> recognition_nn_xout;
  // Fast descriptors (resize)
  unsigned int rgb_person_reid_fast_nn_in_x_res = 16;
  unsigned int rgb_person_reid_fast_nn_in_y_res = 32;
  std::shared_ptr<dai::node::Script> person_config_manip_for_fast_desc_script;
  std::shared_ptr<dai::node::ImageManip> fast_desc_manip;
  std::shared_ptr<dai::node::XLinkOut> fast_desc_xout;
  // Tracking
  std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
  // Visualization
  LazyVideoWriter videoWriter{"", 7.0, 4294967295};

  std::shared_ptr<dai::DeviceInfo> device_info;
  std::shared_ptr<dai::Device> device;

  std::unordered_map<std::string, std::shared_ptr<dai::DataOutputQueue>> queues;
  std::shared_ptr<dai::DataInputQueue> control_queue;

  TwoStageHostSeqSync sync;
  std::vector<std::vector<float>> results;

  // Tracker variables
  TrackerParams params;
  std::unique_ptr<PedestrianTracker> tracker;

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
  std::string model_person_detection_path;
  std::string model_person_detection_path2;
  std::string model_face_detection_path;
  std::string model_face_detection_path2;
  std::string model_face_detection_proc_path;
  std::string model_face_detection_proc_path2;
  std::string model_depth_diff_path;
  std::string model_depth_diff_path2;
  std::string model_reid_path;
  std::string model_reid_path2;

  std::vector<std::vector<float>> reid_results;

  std::string rgb_and_depth_out_script_string = R"(
        import time
        import json

        send_rgb_as_they_come = False
        send_depth_as_they_come = False

        while True:
            time.sleep(0.001)
            command = node.io['commands'].tryGet()
            if command is not None:
                node.warn('rgb_and_depth_out_script received command')
                data = command.getData()
                jsonStr = str(data, 'utf-8')
                dict = json.loads(jsonStr)
                commandString = dict['message']
                node.warn('message was: ' + commandString)
                if commandString == "send_rgb_as_they_come":
                    send_rgb_as_they_come = True
                elif commandString == "send_depth_as_they_come":
                    send_depth_as_they_come = True  
                elif commandString == "stop_send_rgb":
                    send_rgb_as_they_come = False
                elif commandString == "stop_send_depth":
                    send_depth_as_they_come = False

            rgb_frame = node.io['rgb'].tryGet()
            if rgb_frame is not None:
                if send_rgb_as_they_come:
                    node.io['rgb_out'].send(rgb_frame)

            depth_frame = node.io['depth'].tryGet()  
            if depth_frame is not None:
                if send_depth_as_they_come:
                    node.io['depth_out'].send(depth_frame)
    )";

  std::string createReidConfigScript() {
    std::ostringstream reid_config_script;
        reid_config_script << R"(
        import time
        msgs = dict()
        def add_msg(msg, name, seq = None):
            global msgs
            if seq is None:
                seq = msg.getSequenceNum()
            seq = str(seq)
            if seq not in msgs:
                # node.warn(f"Recving image seq num: {seq}")
                msgs[seq] = dict()
            msgs[seq][name] = msg
            if 15 < len(msgs):
                # node.warn(f"Removing first element! len {len(msgs)}")
                # Check all items in msgs and print incomplete ones
                for seq, syncMsgs in msgs.items():
                    if len(syncMsgs) != 2:
                        missing = 'preview' if 'dets' in syncMsgs else 'dets'
                        # node.warn(f"Incomplete set in seq {seq}: missing {missing}")
                # Pop the item and print its sequence number
                popped_item = msgs.popitem()
                # node.warn(f"                       Removed seq: {popped_item[0]}")
        def get_msgs():
            global msgs
            for seq, syncMsgs in msgs.items():
                if len(syncMsgs) == 2:
                    result = syncMsgs
                    del msgs[seq]
                    return result
            return None
        def correct_bb(bb):
            if bb.xmin < 0: bb.xmin = 0.001
            if bb.ymin < 0: bb.ymin = 0.001
            if bb.xmax > 1: bb.xmax = 0.999
            if bb.ymax > 1: bb.ymax = 0.999
            return bb
        while True:
            time.sleep(0.001)
            preview = node.io['preview'].tryGet()
            if preview is not None:
                add_msg(preview, 'preview')
            dets = node.io['person_dets_in'].tryGet()
            if dets is not None:
                seq = dets.getSequenceNum()
                add_msg(dets, 'dets', seq)
            sync_msgs = get_msgs()
            if sync_msgs is not None:
                img = sync_msgs['preview']
                dets = sync_msgs['dets']
                dets_seq = dets.getSequenceNum()
                img_seq = img.getSequenceNum()
                # node.warn(f"   Sending image seq num: {img_seq}")
                for i, det in enumerate(dets.detections):
                    cfg = ImageManipConfig()
                    correct_bb(det)
                    cfg.setCropRect(det.xmin, det.ymin, det.xmax, det.ymax)
                    cfg.setResize()";
        reid_config_script << std::to_string(rgb_person_reid_strong_nn_in_x_res) << ", " << std::to_string(rgb_person_reid_strong_nn_in_y_res) << ")";
        reid_config_script << R"(
                    cfg.setKeepAspectRatio(False)
                    node.io['manip_cfg'].send(cfg)
                    node.io['manip_img'].send(img))";
    return reid_config_script.str();
  };

    std::string createFastDescConfigScript() {
      std::ostringstream fast_desc_script;
          fast_desc_script << R"(
          import time
          msgs = dict()
          def add_msg(msg, name, seq = None):
              global msgs
              if seq is None:
                  seq = msg.getSequenceNum()
              seq = str(seq)
              if seq not in msgs:
                  msgs[seq] = dict()
              msgs[seq][name] = msg
              if 15 < len(msgs):
                  # node.warn(f"Removing first element! len {len(msgs)}")
                  msgs.popitem()
          def get_msgs():
              global msgs
              seq_remove = []
              for seq, syncMsgs in msgs.items():
                  seq_remove.append(seq)
                  if len(syncMsgs) == 2:
                      for rm in seq_remove:
                          del msgs[rm]
                      return syncMsgs
              return None
          def correct_bb(bb):
              if bb.xmin < 0: bb.xmin = 0.001
              if bb.ymin < 0: bb.ymin = 0.001
              if bb.xmax > 1: bb.xmax = 0.999
              if bb.ymax > 1: bb.ymax = 0.999
              return bb
          while True:
              time.sleep(0.001)
              preview = node.io['preview'].tryGet()
              if preview is not None:
                  add_msg(preview, 'preview')
              dets = node.io['person_dets_in'].tryGet()
              if dets is not None:
                  seq = dets.getSequenceNum()
                  add_msg(dets, 'dets', seq)
              sync_msgs = get_msgs()
              if sync_msgs is not None:
                  img = sync_msgs['preview']
                  dets = sync_msgs['dets']
                  dets_seq = dets.getSequenceNum()
                  img_seq = img.getSequenceNum()
                  for i, det in enumerate(dets.detections):
                      cfg = ImageManipConfig()
                      correct_bb(det)
                      cfg.setCropRect(det.xmin, det.ymin, det.xmax, det.ymax)
                      cfg.setResize()";
          fast_desc_script << std::to_string(rgb_person_reid_fast_nn_in_x_res) << ", " << std::to_string(rgb_person_reid_fast_nn_in_y_res) << ")";
          fast_desc_script << R"(
                      cfg.setKeepAspectRatio(False)
                      node.io['manip_cfg'].send(cfg)
                      node.io['manip_img'].send(img))";
    return fast_desc_script.str();
  };

public:
  OakdXlinkFullReader(YAML::Node config, const char* client_key = nullptr, const char* environment_name = nullptr, const char* sensor_name = nullptr);

  ~OakdXlinkFullReader();

  void Reset();

  bool HasNextFrame();

  void NextFrame(const std::vector<std::string> frame_types_to_pull = {});

  std::vector<std::shared_ptr<FrameStruct>> GetCurrentFrame();

  unsigned int GetCurrentFrameId();

  virtual void GoToFrame(unsigned int frame_id);

  unsigned int GetFps();

  std::vector<FrameType> GetType();
};

} // namespace