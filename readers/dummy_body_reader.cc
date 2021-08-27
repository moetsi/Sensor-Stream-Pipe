//
// Created by amourao on 26-06-2019.
//

#include "dummy_body_reader.h"


DummyBodyReader::DummyBodyReader() {
  current_frame_counter_ = 0;

  frame_template_.sensor_id = 0;
  frame_template_.stream_id = RandomString(16);
  frame_template_.device_id = 0;
  frame_template_.scene_desc = "dummybody";

  frame_template_.frame_id = 0;
  frame_template_.message_type = 0;

  frame_template_.frame_type = 4;
  frame_template_.frame_data_type = 8;

}

DummyBodyReader::~DummyBodyReader() {
}

void DummyBodyReader::NextFrame() {

  current_frame_.clear();


  uint64_t capture_timestamp = CurrentTimeMs();
    std::shared_ptr<FrameStruct> s =
        std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
    s->frame_id = current_frame_counter_++;
    s->timestamps.push_back(capture_timestamp);

    s->frame = std::vector<uchar>();
    s->frame.resize(sizeof(_object_human_t) + sizeof(int));
    //here we will say we detected 1 body
    int bodyCount = 1;
    _object_human_t bodyStruct;

    bodyStruct.Id = 1;
    bodyStruct.pelvis_x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    // bodyStruct.pelvis_x = 1.4;
    bodyStruct.pelvis_y = 0.1;
    bodyStruct.pelvis_z = 0.1;
    bodyStruct.pelvis_QX = 0.1;
    bodyStruct.pelvis_QY = 0.1;
    bodyStruct.pelvis_QZ = 0.1;
    bodyStruct.pelvis_QW = 0.1;
    bodyStruct.pelvis_conf = 1;
    bodyStruct.spine_navel_x = 0.1;
    bodyStruct.spine_navel_y = 0.1;
    bodyStruct.spine_navel_z = 0.1;
    bodyStruct.spine_navel_QX = 0.1;
    bodyStruct.spine_navel_QY = 0.1;
    bodyStruct.spine_navel_QZ = 0.1;
    bodyStruct.spine_navel_QW = 0.1;
    bodyStruct.spine_navel_conf = 1;
    bodyStruct.spine_chest_x = 0.1;
    bodyStruct.spine_chest_y = 0.1;
    bodyStruct.spine_chest_z = 0.1;
    bodyStruct.spine_chest_QX = 0.1;
    bodyStruct.spine_chest_QY = 0.1;
    bodyStruct.spine_chest_QZ = 0.1;
    bodyStruct.spine_chest_QW = 0.1;
    bodyStruct.spine_chest_conf = 1;
    bodyStruct.neck_x = 0.1;
    bodyStruct.neck_y = 0.1;
    bodyStruct.neck_z = 0.1;
    bodyStruct.neck_QX = 0.1;
    bodyStruct.neck_QY = 0.1;
    bodyStruct.neck_QZ = 0.1;
    bodyStruct.neck_QW = 0.1;
    bodyStruct.neck_conf = 1;
    bodyStruct.clavicle_left_x = 0.1;
    bodyStruct.clavicle_left_y = 0.1;
    bodyStruct.clavicle_left_z = 0.1;
    bodyStruct.clavicle_left_QX = 0.1;
    bodyStruct.clavicle_left_QY = 0.1;
    bodyStruct.clavicle_left_QZ = 0.1;
    bodyStruct.clavicle_left_QW = 0.1;
    bodyStruct.clavicle_left_conf = 1;
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
    bodyStruct.hand_left_x = 0.1;
    bodyStruct.hand_left_y = 0.1;
    bodyStruct.hand_left_z = 0.1;
    bodyStruct.hand_left_QX = 0.1;
    bodyStruct.hand_left_QY = 0.1;
    bodyStruct.hand_left_QZ = 0.1;
    bodyStruct.hand_left_QW = 0.1;
    bodyStruct.hand_left_conf = 1;
    bodyStruct.handtip_left_x = 0.1;
    bodyStruct.handtip_left_y = 0.1;
    bodyStruct.handtip_left_z = 0.1;
    bodyStruct.handtip_left_QX = 0.1;
    bodyStruct.handtip_left_QY = 0.1;
    bodyStruct.handtip_left_QZ = 0.1;
    bodyStruct.handtip_left_QW = 0.1;
    bodyStruct.handtip_left_conf = 1;
    bodyStruct.thumb_left_x = 0.1;
    bodyStruct.thumb_left_y = 0.1;
    bodyStruct.thumb_left_z = 0.1;
    bodyStruct.thumb_left_QX = 0.1;
    bodyStruct.thumb_left_QY = 0.1;
    bodyStruct.thumb_left_QZ = 0.1;
    bodyStruct.thumb_left_QW = 0.1;
    bodyStruct.thumb_left_conf = 1;
    bodyStruct.clavicle_right_x = 0.1;
    bodyStruct.clavicle_right_y = 0.1;
    bodyStruct.clavicle_right_z = 0.1;
    bodyStruct.clavicle_right_QX = 0.1;
    bodyStruct.clavicle_right_QY = 0.1;
    bodyStruct.clavicle_right_QZ = 0.1;
    bodyStruct.clavicle_right_QW = 0.1;
    bodyStruct.clavicle_right_conf = 1;
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
    bodyStruct.hand_right_x = 0.1;
    bodyStruct.hand_right_y = 0.1;
    bodyStruct.hand_right_z = 0.1;
    bodyStruct.hand_right_QX = 0.1;
    bodyStruct.hand_right_QY = 0.1;
    bodyStruct.hand_right_QZ = 0.1;
    bodyStruct.hand_right_QW = 0.1;
    bodyStruct.hand_right_conf = 1;
    bodyStruct.handtip_right_x = 0.1;
    bodyStruct.handtip_right_y = 0.1;
    bodyStruct.handtip_right_z = 0.1;
    bodyStruct.handtip_right_QX = 0.1;
    bodyStruct.handtip_right_QY = 0.1;
    bodyStruct.handtip_right_QZ = 0.1;
    bodyStruct.handtip_right_QW = 0.1;
    bodyStruct.handtip_right_conf = 1;
    bodyStruct.thumb_right_x = 0.1;
    bodyStruct.thumb_right_y = 0.1;
    bodyStruct.thumb_right_z = 0.1;
    bodyStruct.thumb_right_QX = 0.1;
    bodyStruct.thumb_right_QY = 0.1;
    bodyStruct.thumb_right_QZ = 0.1;
    bodyStruct.thumb_right_QW = 0.1;
    bodyStruct.thumb_right_conf = 1;
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
    bodyStruct.foot_left_x = 0.1;
    bodyStruct.foot_left_y = 0.1;
    bodyStruct.foot_left_z = 0.1;
    bodyStruct.foot_left_QX = 0.1;
    bodyStruct.foot_left_QY = 0.1;
    bodyStruct.foot_left_QZ = 0.1;
    bodyStruct.foot_left_QW = 0.1;
    bodyStruct.foot_left_conf = 1;
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
    bodyStruct.foot_right_x = 0.1;
    bodyStruct.foot_right_y = 0.1;
    bodyStruct.foot_right_z = 0.1;
    bodyStruct.foot_right_QX = 0.1;
    bodyStruct.foot_right_QY = 0.1;
    bodyStruct.foot_right_QZ = 0.1;
    bodyStruct.foot_right_QW = 0.1;
    bodyStruct.foot_right_conf = 1;
    bodyStruct.head_x = 0.1;
    bodyStruct.head_y = 0.1;
    bodyStruct.head_z = 0.1;
    bodyStruct.head_QX = 0.1;
    bodyStruct.head_QY = 0.1;
    bodyStruct.head_QZ = 0.1;
    bodyStruct.head_QW = 0.1;
    bodyStruct.head_conf = 1;
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


    memcpy(&s->frame[0], &bodyCount, sizeof(int));
    memcpy(&s->frame[4], &bodyStruct, sizeof(_object_human_t));


    current_frame_.push_back(s);
}

bool DummyBodyReader::HasNextFrame() { return true; }

void DummyBodyReader::Reset() {}

std::vector<std::shared_ptr<FrameStruct>> DummyBodyReader::GetCurrentFrame() {
  return current_frame_;
}

unsigned int DummyBodyReader::GetFps() {
  return 10;
}

std::vector<unsigned int> DummyBodyReader::GetType() {
  std::vector<unsigned int> types;

  types.push_back(4);
  return types;
}

void DummyBodyReader::GoToFrame(unsigned int frame_id) {}
unsigned int DummyBodyReader::GetCurrentFrameId() { return current_frame_counter_; }
