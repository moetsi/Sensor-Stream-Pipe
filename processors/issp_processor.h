//
// Created by amourao on 23-10-2019.
//

#pragma once

#include "../structs/frame_struct.hpp"

class ISSPProcessor {

public:
  virtual ~ISSPProcessor() {}
  virtual void Start() = 0;
  virtual void Stop() = 0;

  virtual void AddFilter(std::string filter) = 0;
  virtual void ClearFilter() = 0;
};
