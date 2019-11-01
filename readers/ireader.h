//
// Created by amourao on 14-08-2019.
//

#pragma once

#include "../structs/frame_struct.hpp"

class IReader {

public:
  virtual ~IReader() {}

  virtual std::vector<std::shared_ptr<FrameStruct>> GetCurrentFrame() = 0;

  virtual std::vector<unsigned int> GetType() = 0;

  virtual bool HasNextFrame() = 0;

  virtual void NextFrame() = 0;

  virtual void Reset() = 0;

  virtual void GoToFrame(unsigned int frame_id) = 0;

  virtual unsigned int GetCurrentFrameId() = 0;

  virtual unsigned int GetFps() = 0;
};
