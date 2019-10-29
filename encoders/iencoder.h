//
// Created by amourao on 12-09-2019.
//

#pragma once

#include "../structs/frame_struct.hpp"

class IEncoder {

public:
  virtual ~IEncoder() {}

  virtual void AddFrameStruct(std::shared_ptr<FrameStruct> &frame_struct) = 0;

  virtual void NextPacket() = 0;

  virtual bool HasNextPacket() = 0;

  virtual std::shared_ptr<FrameStruct> CurrentFrameEncoded() = 0;

  virtual std::shared_ptr<FrameStruct> CurrentFrameOriginal() = 0;

  virtual std::shared_ptr<CodecParamsStruct> GetCodecParamsStruct() = 0;

  virtual unsigned int GetFps() = 0;
};