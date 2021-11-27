/**
 * \file multi_image_reader.cc @brief Multi image reader
 */
// Created by amourao on 26-06-2019.
#include "multi_image_reader.h"

namespace moetsi::ssp {

MultiImageReader::MultiImageReader(std::vector<std::string> filenames) {

  for(std::string filename: filenames){
    std::shared_ptr<IReader> reader = std::shared_ptr<ImageReader>(new ImageReader(filename));
    reader->Reset();
    readers_.push_back(reader);
  }
  for(auto reader: readers_)
    for(auto s: reader->GetCurrentFrame())
      current_frame_internal_.push_back(s);

}

MultiImageReader::~MultiImageReader() {
}

unsigned int MultiImageReader::GetCurrentFrameId() { return frame_counter_; }

std::vector<std::shared_ptr<FrameStruct>> MultiImageReader::GetCurrentFrame() {
  return current_frame_internal_;
}

void MultiImageReader::NextFrame() {
  frame_counter_ += 1;
  current_frame_internal_.clear();
  for(auto reader: readers_){
    reader->NextFrame();

    for(auto s: reader->GetCurrentFrame())
      current_frame_internal_.push_back(s);
  }
}

bool MultiImageReader::HasNextFrame() {
  for(auto reader: readers_)
    if (!reader->HasNextFrame())
      return false;
  return true;
}

void MultiImageReader::GoToFrame(unsigned int frame_id) {
  for(auto reader: readers_)
    reader->GoToFrame(frame_id);
}

void MultiImageReader::Reset() {
  for(auto reader: readers_)
    reader->Reset();
}

unsigned int MultiImageReader::GetFps() { return readers_.front()->GetFps(); }

std::vector<FrameType> MultiImageReader::GetType() {
  std::vector<FrameType> result;
  for(auto s: current_frame_internal_)
    result.push_back(s->frame_type);
  return result;
}

} // namespace moetsi::ssp
