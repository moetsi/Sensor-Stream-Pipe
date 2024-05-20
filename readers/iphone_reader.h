/**
 * \file iphone_reader.h @brief iPhone driver
 */
// Created by David Geldreich on 01-06-2021.

#pragma once

#include <vector>

#include "../structs/frame_struct.h"
#include "ireader.h"

namespace moetsi::ssp {

class iPhoneReaderImpl;

class iPhoneReader : public IReader {
public:
  iPhoneReader(unsigned int fps = 5); // Constructor with fps parameter
  ~iPhoneReader();

  void Reset() override;
  bool HasNextFrame() override;
  void NextFrame(const std::vector<std::string> frame_types_to_pull = {}) override;
  std::vector<std::shared_ptr<FrameStruct>> GetCurrentFrame() override;
  unsigned int GetCurrentFrameId() override;
  void GoToFrame(unsigned int frame_id) override;
  unsigned int GetFps() override;
  std::vector<FrameType> GetType() override;
    
private:
  unsigned int fps_;
  iPhoneReaderImpl* pImpl;
  FrameStruct frame_template_;
};

}