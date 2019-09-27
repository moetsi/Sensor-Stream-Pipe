//
// Created by amourao on 27-09-2019.
//

#pragma once

#include <zmq.hpp>

#include "../decoders/IDecoder.h"
#include "../structs/FrameStruct.hpp"
#include "IReader.h"

class NetworkReader {

private:
  uint64_t last_time = currentTimeMs();
  uint64_t start_time = last_time;
  uint64_t rec_frames = 0;
  double rec_mbytes = 0;

  int currentFrameCounter;

  std::unordered_map<std::string, double> rec_mbytes_per_stream;
  std::vector<FrameStruct> currentFrameInternal;

  int port;
  zmq::context_t *context;
  zmq::socket_t *socket;

public:
  NetworkReader(int port);
  void init();

  ~NetworkReader();

  void reset();

  void goToFrame(unsigned int frameId);

  bool hasNextFrame();

  void nextFrame();

  std::vector<uint> getType();

  std::vector<FrameStruct> currentFrame();

  uint currentFrameId();

  unsigned int getFps();
};
