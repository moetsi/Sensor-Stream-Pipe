//
// Created by amourao on 16/09/19.
//

#pragma once

#include "iencoder.h"
class NullEncoder : public IEncoder {
private:
  FrameStruct *frame;
  unsigned int fps;

public:
  NullEncoder(int _fps);

  ~NullEncoder();

  void AddFrameStruct(FrameStruct *fs);

  void NextPacket();

  bool HasNextPacket();

  FrameStruct *CurrentFrameEncoded();

  FrameStruct *CurrentFrameOriginal();

  CodecParamsStruct *GetCodecParamsStruct();

  unsigned int GetFps();
};
