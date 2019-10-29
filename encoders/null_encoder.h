//
// Created by amourao on 16/09/19.
//

#pragma once

#include "iencoder.h"
class NullEncoder : public IEncoder {
private:
  std::shared_ptr<FrameStruct> frame;
  unsigned int fps;

public:
  NullEncoder(int _fps);

  ~NullEncoder();

  void AddFrameStruct(std::shared_ptr<FrameStruct> &fs);

  void NextPacket();

  bool HasNextPacket();

  std::shared_ptr<FrameStruct> CurrentFrameEncoded();

  std::shared_ptr<FrameStruct> CurrentFrameOriginal();

  std::shared_ptr<CodecParamsStruct> GetCodecParamsStruct();

  unsigned int GetFps();
};
