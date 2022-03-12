//
// Created by adammpolak on 26-08-2021.
//

/**
 * \file dummy_body_reader.cc @brief Dumy Body Reader
 */
#include "dummy_body_reader.h"

namespace moetsi::ssp {

DummyBodyReader::DummyBodyReader(YAML::Node config) {
  current_frame_counter_ = 0;

  frame_template_.sensor_id = 0;
  frame_template_.stream_id = RandomString(16);
  frame_template_.device_id = 0;
  frame_template_.scene_desc = "dummybody";

  frame_template_.frame_id = 0;
  frame_template_.message_type = SSPMessageType::MessageTypeDefault; // 0;

  frame_template_.frame_type = FrameType::FrameTypeHumanPose; // 4;
  frame_template_.frame_data_type = FrameDataType::FrameDataTypeObjectHumanData; // 8;

  int nhumans_ = config["nhumans"].as<int>();
  int stopAt_ = config["stopat"].as<int>();
  std::cerr << ((void*)this) << " nhumans_ = " << nhumans_ << " stopAt_ = " << stopAt_ << std::endl << std::flush;
}

DummyBodyReader::~DummyBodyReader() {
}

void DummyBodyReader::NextFrame() {
  std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
  std::this_thread::sleep_for(std::chrono::milliseconds(1024 / 100)); // 100 fps
  current_frame_.clear();

  uint64_t capture_timestamp = CurrentTimeMs();
  std::shared_ptr<FrameStruct> s =
      std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
  std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;  
  s->frame_id = current_frame_counter_++;
  s->timestamps.push_back(capture_timestamp);
  s->frame = std::vector<uchar>();
  std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;  
  std::cerr << ((void*)this) << " " << nhumans_  << " " << (sizeof(coco_human_t) * nhumans_ + sizeof(int32_t)) << std::endl << std::flush;
  s->frame.resize(sizeof(coco_human_t) * nhumans_ + sizeof(int32_t));
  std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
  //here we will say we detected 1 body
  int32_t bodyCount = (int32_t) nhumans_;
  coco_human_t bodyStruct;

  bodyStruct.Id = 1;
  inplace_hton(bodyStruct.Id);
  bodyStruct.pelvis_x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
  // bodyStruct.pelvis_x = 1.4;
  bodyStruct.pelvis_y = 0.1;
  bodyStruct.pelvis_z = 0.1;
  bodyStruct.pelvis_QX = 0.1;
  bodyStruct.pelvis_QY = 0.1;
  bodyStruct.pelvis_QZ = 0.1;
  bodyStruct.pelvis_QW = 0.1;
  bodyStruct.pelvis_conf = 1;
  bodyStruct.neck_x = 0.1;
  bodyStruct.neck_y = 0.1;
  bodyStruct.neck_z = 0.1;
  bodyStruct.neck_QX = 0.1;
  bodyStruct.neck_QY = 0.1;
  bodyStruct.neck_QZ = 0.1;
  bodyStruct.neck_QW = 0.1;
  bodyStruct.neck_conf = 1;
  bodyStruct.shoulder_left_x = 0.1;
  bodyStruct.shoulder_left_y = 0.1;
  bodyStruct.shoulder_left_z = 0.1;
  bodyStruct.shoulder_left_QX = 0.1;
  bodyStruct.shoulder_left_QY = 0.1;
  bodyStruct.shoulder_left_QZ = 0.1;
  bodyStruct.shoulder_left_QW = 0.1;
  bodyStruct.shoulder_left_conf = 1;
  bodyStruct.elbow_left_x = 0.1;
  bodyStruct.elbow_left_y = 0.1;
  bodyStruct.elbow_left_z = 0.1;
  bodyStruct.elbow_left_QX = 0.1;
  bodyStruct.elbow_left_QY = 0.1;
  bodyStruct.elbow_left_QZ = 0.1;
  bodyStruct.elbow_left_QW = 0.1;
  bodyStruct.elbow_left_conf = 1;
  bodyStruct.wrist_left_x = 0.1;
  bodyStruct.wrist_left_y = 0.1;
  bodyStruct.wrist_left_z = 0.1;
  bodyStruct.wrist_left_QX = 0.1;
  bodyStruct.wrist_left_QY = 0.1;
  bodyStruct.wrist_left_QZ = 0.1;
  bodyStruct.wrist_left_QW = 0.1;
  bodyStruct.wrist_left_conf = 1;
  bodyStruct.shoulder_right_x = 0.1;
  bodyStruct.shoulder_right_y = 0.1;
  bodyStruct.shoulder_right_z = 0.1;
  bodyStruct.shoulder_right_QX = 0.1;
  bodyStruct.shoulder_right_QY = 0.1;
  bodyStruct.shoulder_right_QZ = 0.1;
  bodyStruct.shoulder_right_QW = 0.1;
  bodyStruct.shoulder_right_conf = 1;
  bodyStruct.elbow_right_x = 0.1;
  bodyStruct.elbow_right_y = 0.1;
  bodyStruct.elbow_right_z = 0.1;
  bodyStruct.elbow_right_QX = 0.1;
  bodyStruct.elbow_right_QY = 0.1;
  bodyStruct.elbow_right_QZ = 0.1;
  bodyStruct.elbow_right_QW = 0.1;
  bodyStruct.elbow_right_conf = 1;
  bodyStruct.wrist_right_x = 0.1;
  bodyStruct.wrist_right_y = 0.1;
  bodyStruct.wrist_right_z = 0.1;
  bodyStruct.wrist_right_QX = 0.1;
  bodyStruct.wrist_right_QY = 0.1;
  bodyStruct.wrist_right_QZ = 0.1;
  bodyStruct.wrist_right_QW = 0.1;
  bodyStruct.wrist_right_conf = 1;
  bodyStruct.hip_left_x = 0.1;
  bodyStruct.hip_left_y = 0.1;
  bodyStruct.hip_left_z = 0.1;
  bodyStruct.hip_left_QX = 0.1;
  bodyStruct.hip_left_QY = 0.1;
  bodyStruct.hip_left_QZ = 0.1;
  bodyStruct.hip_left_QW = 0.1;
  bodyStruct.hip_left_conf = 1;
  bodyStruct.knee_left_x = 0.1;
  bodyStruct.knee_left_y = 0.1;
  bodyStruct.knee_left_z = 0.1;
  bodyStruct.knee_left_QX = 0.1;
  bodyStruct.knee_left_QY = 0.1;
  bodyStruct.knee_left_QZ = 0.1;
  bodyStruct.knee_left_QW = 0.1;
  bodyStruct.knee_left_conf = 1;
  bodyStruct.ankle_left_x = 0.1;
  bodyStruct.ankle_left_y = 0.1;
  bodyStruct.ankle_left_z = 0.1;
  bodyStruct.ankle_left_QX = 0.1;
  bodyStruct.ankle_left_QY = 0.1;
  bodyStruct.ankle_left_QZ = 0.1;
  bodyStruct.ankle_left_QW = 0.1;
  bodyStruct.ankle_left_conf = 1;
  bodyStruct.hip_right_x = 0.1;
  bodyStruct.hip_right_y = 0.1;
  bodyStruct.hip_right_z = 0.1;
  bodyStruct.hip_right_QX = 0.1;
  bodyStruct.hip_right_QY = 0.1;
  bodyStruct.hip_right_QZ = 0.1;
  bodyStruct.hip_right_QW = 0.1;
  bodyStruct.hip_right_conf = 1;
  bodyStruct.knee_right_x = 0.1;
  bodyStruct.knee_right_y = 0.1;
  bodyStruct.knee_right_z = 0.1;
  bodyStruct.knee_right_QX = 0.1;
  bodyStruct.knee_right_QY = 0.1;
  bodyStruct.knee_right_QZ = 0.1;
  bodyStruct.knee_right_QW = 0.1;
  bodyStruct.knee_right_conf = 1;
  bodyStruct.ankle_right_x = 0.1;
  bodyStruct.ankle_right_y = 0.1;
  bodyStruct.ankle_right_z = 0.1;
  bodyStruct.ankle_right_QX = 0.1;
  bodyStruct.ankle_right_QY = 0.1;
  bodyStruct.ankle_right_QZ = 0.1;
  bodyStruct.ankle_right_QW = 0.1;
  bodyStruct.ankle_right_conf = 1;
  bodyStruct.nose_x = 0.1;
  bodyStruct.nose_y = 0.1;
  bodyStruct.nose_z = 0.1;
  bodyStruct.nose_QX = 0.1;
  bodyStruct.nose_QY = 0.1;
  bodyStruct.nose_QZ = 0.1;
  bodyStruct.nose_QW = 0.1;
  bodyStruct.nose_conf = 1;
  bodyStruct.eye_left_x = 0.1;
  bodyStruct.eye_left_y = 0.1;
  bodyStruct.eye_left_z = 0.1;
  bodyStruct.eye_left_QX = 0.1;
  bodyStruct.eye_left_QY = 0.1;
  bodyStruct.eye_left_QZ = 0.1;
  bodyStruct.eye_left_QW = 0.1;
  bodyStruct.eye_left_conf = 1;
  bodyStruct.ear_left_x = 0.1;
  bodyStruct.ear_left_y = 0.1;
  bodyStruct.ear_left_z = 0.1;
  bodyStruct.ear_left_QX = 0.1;
  bodyStruct.ear_left_QY = 0.1;
  bodyStruct.ear_left_QZ = 0.1;
  bodyStruct.ear_left_QW = 0.1;
  bodyStruct.ear_left_conf = 1;
  bodyStruct.eye_right_x = 0.1;
  bodyStruct.eye_right_y = 0.1;
  bodyStruct.eye_right_z = 0.1;
  bodyStruct.eye_right_QX = 0.1;
  bodyStruct.eye_right_QY = 0.1;
  bodyStruct.eye_right_QZ = 0.1;
  bodyStruct.eye_right_QW = 0.1;
  bodyStruct.eye_right_conf = 1;
  bodyStruct.ear_right_x = 0.1;
  bodyStruct.ear_right_y = 0.1;
  bodyStruct.ear_right_z = 0.1;
  bodyStruct.ear_right_QX = 0.1;
  bodyStruct.ear_right_QY = 0.1;
  bodyStruct.ear_right_QZ = 0.1;
  bodyStruct.ear_right_QW = 0.1;
  bodyStruct.ear_right_conf = 1;
  inplace_hton(bodyCount);
  // std::cerr << "dummy: bodyCount after hton " << bodyCount << std::endl << std::flush;
  std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
  memcpy(&s->frame[0], &bodyCount, sizeof(int32_t));
  for (int i=0; i < nhumans_; ++i) {
    ////std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    bodyStruct.Id = (i+1);
    inplace_hton(bodyStruct.Id);
    memcpy(&s->frame[4 + sizeof(coco_human_t)*i], &bodyStruct, sizeof(coco_human_t));
    ///std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
  }
  ////std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
  current_frame_.push_back(s);

  if (counter_ ++ > this->stopAt_) {
    has_next_ = false;
  }
}

bool DummyBodyReader::HasNextFrame() { 
  return has_next_;
  // ** return true; 
}

void DummyBodyReader::Reset() {}

std::vector<std::shared_ptr<FrameStruct>> DummyBodyReader::GetCurrentFrame() {
  return current_frame_;
}

unsigned int DummyBodyReader::GetFps() {
  return 100;
}

std::vector<FrameType> DummyBodyReader::GetType() {
  std::vector<FrameType> types;
  types.push_back(FrameType::FrameTypeHumanPose);
  return types;
}

void DummyBodyReader::GoToFrame(unsigned int frame_id) {}
unsigned int DummyBodyReader::GetCurrentFrameId() { return current_frame_counter_; }

} // namespace moetsi::ssp
