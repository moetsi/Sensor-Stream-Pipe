//
// Created by amourao on 26-06-2019.
//

#include <chrono>
#include <iostream>
#include <thread>
#include <unistd.h>

#include <k4a/k4a.h>
#include <opencv2/imgproc.hpp>
#include <zmq.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

#include "../utils/Logger.h"

#include "../decoders/IDecoder.h"
#include "../decoders/LibAvDecoder.h"
#include "../decoders/NvDecoder.h"
#include "../readers/ImageReader.h"
#include "../structs/FrameStruct.hpp"
#include "../utils/Utils.h"
#include "../utils/VideoUtils.h"

int main(int argc, char *argv[]) {

  spdlog::set_level(spdlog::level::debug);

  srand(time(NULL) * getpid());

  try {

    if (argc < 2) {
      std::cerr << "Usage: ssp_client <port> (<log level>) (<log file>)"
                << std::endl;
      return 1;
    }
    std::string log_level = "debug";
    std::string log_file = "";

    if (argc > 2)
      log_level = argv[2];
    if (argc > 3)
      log_file = argv[3];

    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_PULL);
    socket.bind("tcp://*:" + std::string(argv[1]));

    uint64_t last_time = currentTimeMs();
    uint64_t start_time = last_time;
    uint64_t rec_frames = 0;
    double rec_mbytes = 0;

    setupLogging(log_level, log_file);

    std::unordered_map<std::string, double> rec_mbytes_per_stream;

    std::unordered_map<std::string, IDecoder *> decoders;

    bool imgChanged = false;

    for (;;) {
      zmq::message_t request;

      socket.recv(&request);

      if (rec_frames == 0) {
        last_time = currentTimeMs();
        start_time = last_time;
      }

      rec_frames += 1;
      uint64_t diff_time = currentTimeMs() - last_time;
      double diff_start_time =
          (currentTimeMs() - start_time) / (double)rec_frames;
      int64_t avg_fps;
      if (diff_start_time == 0)
        avg_fps = -1;
      else
        avg_fps = 1000 / diff_start_time;

      last_time = currentTimeMs();

      std::string result =
          std::string(static_cast<char *>(request.data()), request.size());

      std::vector<FrameStruct> f_list =
          parseCerealStructFromString<std::vector<FrameStruct>>(result);

      rec_mbytes += request.size() / 1000;

      for (uint i = 0; i < f_list.size(); i++) {
        f_list.at(i).timestamps.push_back(currentTimeMs());
      }
      for (FrameStruct f : f_list) {
        std::string decoder_id = f.streamId + std::to_string(f.sensorId);

        // You can access Kinect camera parameters here
        /*
        if (f.camera_calibration_data.type == 0) {
          k4a_calibration_t *calibration = new k4a_calibration_t();
          k4a_calibration_get_from_raw(
              reinterpret_cast<char *>(f.camera_calibration_data.data.data()),
              f.camera_calibration_data.data.size(),
              static_cast<const k4a_depth_mode_t>(
                  f.camera_calibration_data.extra_data[0]),
              static_cast<const k4a_color_resolution_t>(
                  f.camera_calibration_data.extra_data[1]),
              calibration);
        }
        */

        rec_mbytes_per_stream[decoder_id] += f.frame.size() / 1000;
        cv::Mat img;
        imgChanged = frameStructToMat(f, img, decoders);

        if (imgChanged && !img.empty()) {

          // Manipulate image to show as a color map
          if (f.frameType == 1) {
            if (img.type() == CV_16U) {
              // Compress images to show up on a 255 valued color map
              img *= (MAX_DEPTH_VALUE_8_BITS / (float)MAX_DEPTH_VALUE_12_BITS);
            }
            cv::Mat imgOut;

            img.convertTo(imgOut, CV_8U);
            cv::applyColorMap(imgOut, img, cv::COLORMAP_JET);
          } else if (f.frameType == 2) {

            double max = 1024;
            img *= (MAX_DEPTH_VALUE_8_BITS / (float)max);
            img.convertTo(img, CV_8U);
          }

          cv::namedWindow(decoder_id);
          cv::imshow(decoder_id, img);
          cv::waitKey(1);
          imgChanged = false;
        }
      }

      spdlog::debug(
          "Message received, took {} ms; packet size {}; avg {} fps; {} avg "
          "Mbps; latency: {} ms",
          diff_time, request.size(), avg_fps,
          8 * (rec_mbytes / (currentTimeMs() - start_time)),
          (f_list.front().timestamps.back() - f_list.front().timestamps.at(1)));
      for (FrameStruct f : f_list) {
        std::string decoder_id = f.streamId + std::to_string(f.sensorId);
        spdlog::debug("\t{};{};{} {} avg Mbps; latency: {} ms", f.deviceId,
                      f.sensorId, f.frameId,
                      8 * (rec_mbytes_per_stream[decoder_id] /
                           (currentTimeMs() - start_time)),
                      (f.timestamps.back() - f.timestamps.at(1)));
      }
    }

  } catch (std::exception &e) {
    spdlog::error(e.what());
  }

  return 0;
}
