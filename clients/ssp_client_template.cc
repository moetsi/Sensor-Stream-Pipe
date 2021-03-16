//
// Created by amourao on 26-06-2019.
//

#include <chrono>
#include <iostream>
#include <thread>

#ifdef _WIN32
#include <io.h>
#define SSP_EXPORT __declspec(dllexport)
#else
#include <unistd.h>
#define SSP_EXPORT
#endif

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

#include "../readers/network_reader.h"
#include "../utils/video_utils.h"
#include "../utils/image_converter.h"

extern "C" SSP_EXPORT int ssp_client_template(int port) {
  av_log_set_level(AV_LOG_QUIET);

  try {
    NetworkReader reader(port);

    reader.init();

    std::unordered_map<std::string, std::shared_ptr<IDecoder>> decoders;

    bool imgChanged = false;
    while (reader.HasNextFrame()) {
      reader.NextFrame();
      std::vector<FrameStruct> f_list = reader.GetCurrentFrame();
      for (FrameStruct f : f_list) {
        std::string decoder_id = f.stream_id + std::to_string(f.sensor_id);

        cv::Mat img;
        imgChanged = FrameStructToMat(f, img, decoders);
      }
    }

  } catch (std::exception &e) {
    spdlog::error(e.what());
  }

  return 0;
}

#ifndef SSP_PLUGIN
int main(int argc, char *argv[]) {

  spdlog::set_level(spdlog::level::debug);
  srand(time(NULL));

  if (argc < 2) {
    std::cerr << "Usage: ssp_client_opencv <port> (<log level>) (<log file>)"
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
  return ssp_client_template(port);
}
#endif
