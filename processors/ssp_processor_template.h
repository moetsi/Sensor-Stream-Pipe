//
// Created by amourao on 26-06-2019.
//

#include <chrono>
#include <iostream>
#include <thread>
#include <unistd.h>

#include <opencv2/imgproc.hpp>
#include <zmq.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

#include "../utils/logger.h"

#include "../clients/ssp_coordinator_types.h"
#include "../readers/network_reader.h"
#include "../utils/video_utils.h"

#include "issp_processor.h"

class SSPProcTemplate : public ISSPProcessor {
private:
  std::thread worker_thread_;
  std::string host_port_;
  std::shared_ptr<NetworkReader> reader_;

public:
  SSPProcTemplate(std::string host_port);

  void Start();
  void Stop();

  void AddFilter(std::string filter);
  void ClearFilter();

  // TODO: not happy with this class structure
  static int Worker(std::shared_ptr<NetworkReader> reader) {

    spdlog::set_level(spdlog::level::debug);
    av_log_set_level(AV_LOG_QUIET);

    srand(time(NULL) * getpid());

    try {
      reader->init();

      std::unordered_map<std::string, IDecoder *> decoders;

      bool imgChanged = false;
      while (reader->HasNextFrame()) {
        reader->NextFrame();
        std::vector<FrameStruct> f_list = reader->GetCurrentFrame();
        for (FrameStruct f : f_list) {
          // DO SOMETHING
        }
      }

    } catch (std::exception &e) {
      spdlog::error(e.what());
    }

    return 0;
  }
};
