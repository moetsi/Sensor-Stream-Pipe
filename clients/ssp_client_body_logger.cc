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
#ifdef FFMPEG_AS_FRAMEWORK
#include <FFmpeg/avcodec.h>
#include <FFmpeg/avformat.h>
#include <FFmpeg/avutil.h>
#include <FFmpeg/pixdesc.h>
#include <FFmpeg/swscale.h>
#else
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
#endif
}

#include "../utils/logger.h"

#include "../readers/network_reader.h"
#include "../utils/video_utils.h"
#include "../utils/image_converter.h"

using namespace moetsi::ssp;

// imshow not available on iOS/iPhone Simulator
#if __APPLE__
  #include <TargetConditionals.h>
  #if TARGET_OS_MAC
    #define HAS_IMSHOW 1
  #else
    #define HAS_IMSHOW 0
  #endif
#else
  #define HAS_IMSHOW 1
#endif

extern "C" SSP_EXPORT int ssp_client_body_logger(int port)
{
  av_log_set_level(AV_LOG_QUIET);

  try {
    NetworkReader reader(port);

    reader.init();

    std::unordered_map<std::string, std::shared_ptr<IDecoder>> decoders;

    int32_t bodyCount;
    coco_human_t bodyStruct;

    while (reader.HasNextFrame()) {
      reader.NextFrame();
      std::vector<FrameStruct> f_list = reader.GetCurrentFrame();
      for (FrameStruct f : f_list) {
        if (f.frame_data_type == FrameDataType::FrameDataTypeObjectHumanData)
        {
          std::string decoder_id = f.stream_id + std::to_string(f.sensor_id);

          //First we grab the amount of bodies
          memcpy(&bodyCount, &f.frame[0], sizeof(int32_t));
          inplace_ntoh(bodyCount);

          for (int32_t i=0; i<bodyCount; ++i) {
            //Then we grab the body struct
            // (in the future it will iterate and go over the body struct array)
            memcpy(&bodyStruct, &f.frame[4] + sizeof(coco_human_t) * i, sizeof(coco_human_t));
            bodyStruct.ntoh();
            spdlog::debug("\t description: {} counter: {} bodyStruct's pelvis.x: {} id = {} number of bodies: {}", f.scene_desc, f.frame_id, bodyStruct.pelvis_x, bodyStruct.Id, bodyCount);
          }
        }

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
    
  return ssp_client_body_logger(port);
}
#endif

