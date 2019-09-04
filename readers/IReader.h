//
// Created by amourao on 14-08-2019.
//

#pragma once

#include "../structs/FrameStruct.hpp"

class IReader {

public:
  virtual std::vector<FrameStruct> currentFrame() = 0;

  virtual bool hasNextFrame() = 0;

  virtual void nextFrame() = 0;

  virtual void reset() = 0;

  virtual void goToFrame(uint frameId) = 0;

  virtual uint getFps() = 0;
};
