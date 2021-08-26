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
