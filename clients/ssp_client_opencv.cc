/**
 * \file ssp_client_opencv.cc @brief OpenCV based ssp client client
 */
// Created by amourao on 26-06-2019.

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

using namespace moetsi::ssp;

extern "C" SSP_EXPORT int ssp_client_opencv(int port)
{
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

        if (imgChanged && !img.empty()) {

          // Manipulate image to show as a color map
          // if (f.frame_type == 1) {
          if (f.frame_type == FrameType::FrameTypeDepth) {
            if (img.type() == CV_16U) {
              // Compress images to show up on a 255 valued color map
              img *= (MAX_DEPTH_VALUE_8_BITS / (float)MAX_DEPTH_VALUE_12_BITS);
            } else if (img.type() == CV_32F) {
              // Normalize image to 0;255
              cv::normalize(img, img, 255, 0, cv::NORM_MINMAX);
            }
            cv::Mat imgOut;

            img.convertTo(imgOut, CV_8U);
            cv::applyColorMap(imgOut, img, cv::COLORMAP_JET);
          //} else if (f.frame_type == 2) {
          } else if (f.frame_type == FrameType::FrameTypeIR) {

            double max = 1024;
            img *= (MAX_DEPTH_VALUE_8_BITS / (float)max);
            img.convertTo(img, CV_8U);
          //} else if (f.frame_type == 3) {
          } else if (f.frame_type == FrameType::FrameTypeConfidence) {
            cv::Mat imgOut;
            img *= 127; // iOS confidence is 0:low, 1:medium, 2:high
          }

#if HAS_IMSHOW
          cv::namedWindow(decoder_id);
          cv::imshow(decoder_id, img);
          cv::waitKey(1);
#endif
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
    
  return ssp_client_opencv(port);
}
#endif

