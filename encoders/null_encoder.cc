//
// Created by amourao on 16/09/19.
//
#include "null_encoder.h"

NullEncoder::NullEncoder(int _fps) { fps = _fps; }
NullEncoder::~NullEncoder() {}

void NullEncoder::AddFrameStruct(FrameStruct *fs) { frame = fs; }
void NullEncoder::NextPacket() {}
bool NullEncoder::HasNextPacket() { return frame != nullptr; }
FrameStruct *NullEncoder::CurrentFrameEncoded() { return frame; }
FrameStruct *NullEncoder::CurrentFrameOriginal() { return frame; }
CodecParamsStruct *NullEncoder::GetCodecParamsStruct() { return nullptr; }
unsigned int NullEncoder::GetFps() { return fps; }
