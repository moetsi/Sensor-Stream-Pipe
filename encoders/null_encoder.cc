//
// Created by amourao on 16/09/19.
//
#include "null_encoder.h"

NullEncoder::NullEncoder(int _fps) { fps = _fps; }
NullEncoder::~NullEncoder() {
  if (frame != nullptr)
    delete frame;
}

void NullEncoder::addFrameStruct(FrameStruct *fs) { frame = fs; }
void NullEncoder::nextPacket() {}
bool NullEncoder::hasNextPacket() { return frame != nullptr; }
FrameStruct *NullEncoder::currentFrameEncoded() { return frame; }
FrameStruct *NullEncoder::currentFrameOriginal() { return frame; }
CodecParamsStruct *NullEncoder::getCodecParamsStruct() { return nullptr; }
uint NullEncoder::getFps() { return fps; }
