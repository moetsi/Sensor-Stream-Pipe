/**
 * \file null_encoder.cc @brief Straight pipe encoder
 */
// Created by amourao on 16/09/19.
#include "null_encoder.h"

namespace moetsi::ssp {

NullEncoder::NullEncoder(int _fps) { fps = _fps; }
NullEncoder::~NullEncoder() {}

void NullEncoder::AddFrameStruct(std::shared_ptr<FrameStruct> &fs) {
  frame = fs;
}
void NullEncoder::NextPacket() {}
bool NullEncoder::HasNextPacket() { return frame != nullptr; }
std::shared_ptr<FrameStruct> NullEncoder::CurrentFrameEncoded() {
  return frame;
}
std::shared_ptr<FrameStruct> NullEncoder::CurrentFrameOriginal() {
  return frame;
}
std::shared_ptr<CodecParamsStruct> NullEncoder::GetCodecParamsStruct() {
  return nullptr;
}
unsigned int NullEncoder::GetFps() { return fps; }

} // namespace moetsi::ssp
