//
// Created by amourao on 27-06-2019.
//

#pragma once

#include <atomic>
#include <fstream>
#include <iostream>
#include <vector>

#include "../utils/logger.h"
#include <cereal/archives/binary.hpp>

#include "../structs/frame_struct.hpp"
#include "../structs/body_struct.hpp"
#include "ireader.h"


class DummyBodyReader : public IReader {
private:
  int current_frame_counter_;

  FrameStruct frame_template_;

  std::vector<std::shared_ptr<FrameStruct>> current_frame_;

public:
  DummyBodyReader();

  ~DummyBodyReader();

  void Reset();

  bool HasNextFrame();

  void NextFrame();

  std::vector<std::shared_ptr<FrameStruct>> GetCurrentFrame();

  unsigned int GetCurrentFrameId();

  virtual void GoToFrame(unsigned int frame_id);

  unsigned int GetFps();

  std::vector<unsigned int> GetType();
};
