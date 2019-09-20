//
// Created by amourao on 27-06-2019.
//

#pragma once

#include <fstream>
#include <iostream>
#include <vector>

#include <cereal/archives/binary.hpp>

#include "../structs/FrameStruct.hpp"
#include "IReader.h"

class ImageReader : public IReader {
private:
  unsigned int currentFrameCounter;
  unsigned int fps;
  std::string sceneDesc;
  unsigned int sensorId;
  unsigned int deviceId;
  unsigned int frameType;
  std::string streamId;

  CodecParamsStruct *cps;

  FrameStruct *currentFrameInternal;

  std::vector<unsigned char> readFile(std::string &filename);

  FrameStruct *createFrameStruct(unsigned int frameId);

  std::string getStructBytes(FrameStruct *frame);

  std::vector<std::string> frameLines;

public:
  ImageReader(std::string filename);

  void reset();

  void goToFrame(uint frameId);

  bool hasNextFrame();

  void nextFrame();

  std::vector<FrameStruct *> currentFrame();

  uint currentFrameId();

  std::vector<uint> getType();

  uint getFps();

};
