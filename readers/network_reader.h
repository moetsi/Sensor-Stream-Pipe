//
// Created by amourao on 27-09-2019.
//

#pragma once

#include <zmq.hpp>

#include "../structs/frame_struct.hpp"
#include "ireader.h"

class NetworkReader {

private:
  uint64_t last_time_;
  uint64_t start_time_;
  uint64_t rec_frames_;
  double rec_mbytes_;

  int current_frame_counter_;

  std::unordered_map<std::string, double> rec_mbytes_per_stream_;
  std::vector<FrameStruct> current_frame_internal_;

  std::vector<std::string> subscriptions_;

  std::string host_port_;
  zmq::context_t *context_;
  zmq::socket_t *socket_;

public:
  NetworkReader(std::string host_port);
  void init();

  ~NetworkReader();

  bool HasNextFrame();

  void NextFrame();

  std::vector<FrameStruct> GetCurrentFrame();

  unsigned int GetCurrentFrameId();

  zmq::context_t *GetContext();

  void AddFilter(std::string filter);

  void ResetFilter();

};
