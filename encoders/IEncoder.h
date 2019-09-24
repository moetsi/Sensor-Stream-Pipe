//
// Created by amourao on 12-09-2019.
//

#pragma once

#include "../structs/FrameStruct.hpp"

class IEncoder {

public:
  virtual ~IEncoder() {}

  virtual void addFrameStruct(FrameStruct *fs) = 0;

  virtual void nextPacket() = 0;

  virtual bool hasNextPacket() = 0;

  virtual FrameStruct *currentFrameEncoded() = 0;

  virtual FrameStruct *currentFrameOriginal() = 0;

  virtual CodecParamsStruct *getCodecParamsStruct() = 0;

  virtual uint getFps() = 0;
};