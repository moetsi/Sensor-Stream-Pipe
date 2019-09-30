//
// Created by amourao on 16/09/19.
//

#pragma once

#include "iencoder.h"
class NullEncoder : public IEncoder {
private:
  FrameStruct *frame;
  uint fps;

public:
  NullEncoder(int _fps);

  ~NullEncoder();

  void addFrameStruct(FrameStruct *fs);

  void nextPacket();

  bool hasNextPacket();

  FrameStruct *currentFrameEncoded();

  FrameStruct *currentFrameOriginal();

  CodecParamsStruct *getCodecParamsStruct();

  uint getFps();
};
