//
// Created by amourao on 27-06-2019.
//

#pragma once

#include <fstream>
#include <iostream>
#include <vector>

#include <cereal/archives/binary.hpp>

#include "../structs/frame_struct.hpp"
#include "../utils/image_decoder.h"
#include "ireader.h"

class ImageReader : public IReader {
private:
  unsigned int frame_counter_;
  unsigned int fps_;
  std::string scene_desc_;
  unsigned int sensor_id_;
  unsigned int device_id_;
  unsigned int frame_type_;
  std::string stream_id_;

  CodecParamsStruct *codec_params_struct_;

  FrameStruct *current_frame_internal_;

  std::vector<std::string> frame_lines_;

  std::vector<unsigned char> ReadFile(std::string &filename);

  FrameStruct *CreateFrameStruct(unsigned int frame_id);

public:
  ImageReader(std::string filename);
  ~ImageReader();

  void Reset();

  void GoToFrame(unsigned int frame_id);

  bool HasNextFrame();

  void NextFrame();

  std::vector<FrameStruct *> GetCurrentFrame();

  unsigned int GetCurrentFrameId();

  std::vector<unsigned int> GetType();

  unsigned int GetFps();

  void SetStreamId(const std::string &id);
};
