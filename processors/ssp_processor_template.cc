//
// Created by amourao on 26-06-2019.
//

#include <chrono>
#include <iostream>
#include <thread>
#include <unistd.h>

#include <zmq.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

#include "../utils/logger.h"

#include "../readers/network_reader.h"

int main(int argc, char *argv[]) {

  spdlog::set_level(spdlog::level::debug);
  av_log_set_level(AV_LOG_QUIET);

  srand(time(NULL) * getpid());

  try {

    if (argc < 2) {
      std::cerr
          << "Usage: ssp_client_template <port> (<log level>) (<log file>)"
          << std::endl;
      return 1;
    }
    std::string log_level = "debug";
    std::string log_file = "";

    if (argc > 2)
      log_level = argv[2];
    if (argc > 3)
      log_file = argv[3];

    int port = std::stoi(argv[1]);
    NetworkReader reader(port);

    reader.init();

    while (reader.HasNextFrame()) {
      reader.NextFrame();
      std::vector<FrameStruct> f_list = reader.GetCurrentFrame();
    }

  } catch (std::exception &e) {
    spdlog::error(e.what());
  }

  return 0;
}
