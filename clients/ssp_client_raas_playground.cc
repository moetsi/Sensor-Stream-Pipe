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

struct SensorData {
  double x_position;
  double y_position;
  double z_position;
  int face_id;
  int silhouette_id;
  int this_sensor_body_tracking_id;
};
struct DeviceMessage {
  int device_id;
  double timestamp;
  std::vector<SensorData> sensor_data;
};

std::unordered_map<int, std::tuple<std::chrono::system_clock::time_point, SensorData*, int>> device_message_dictionary;
std::mutex device_message_dictionary_mutex;
std::atomic<bool> stop_thread{false};

extern "C" SSP_EXPORT int ssp_client_opencv(int port, std::string hostname)
{
  av_log_set_level(AV_LOG_QUIET);

  try {

    // We connect to port 9002 on local host which is the RaaS consumer port
    NetworkReader reader(9002);
    reader.init("127.0.0.1");

    std::unordered_map<std::string, std::shared_ptr<IDecoder>> decoders;

    int32_t bodyCount;
    coco_human_t bodyStruct;

    while (reader.HasNextFrame()) {

      coco_human_t bodyStruct;

      reader.NextFrame();
      std::vector<FrameStruct> f_list = reader.GetCurrentFrame();
      for (FrameStruct f : f_list) {

        std::string decoder_id = f.stream_id + std::to_string(f.sensor_id);

        if (f.frame_type == FrameType::FrameTypeHumanPose)
        {
          if (f.frame.size() < 4 + sizeof(coco_human_t)) {

            std::cerr << "0 size frame -> skip " << std::endl << std::flush;
            continue;

          } else {
            // f.device_id is the device_id of the device that sent the message which is an int
            int device_id = f.device_id;
            auto now = std::chrono::system_clock::now();

            // This is dummy data to fill the SensorData struct
            SensorData dummy_sensor_data{0, 0, 0, 0, 0, 0};
            std::vector<SensorData> dummy_sensor_data_vector{dummy_sensor_data};

            std::unique_lock<std::mutex> lock(device_message_dictionary_mutex);
            auto& device_message_entry = device_message_dictionary[device_id];
            std::get<0>(device_message_entry) = now;

            // Allocate memory for the sensor_data array
            SensorData* sensor_data_array = new SensorData[dummy_sensor_data_vector.size()];
            std::copy(dummy_sensor_data_vector.begin(), dummy_sensor_data_vector.end(), sensor_data_array);
            std::get<1>(device_message_entry) = sensor_data_array;
            std::get<2>(device_message_entry) = dummy_sensor_data_vector.size();  // Store the sensor_data_count
            lock.unlock();
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
    std::cerr << "Usage: ssp_client_opencv <port> <host> (<log level>) (<log file>)"
              << std::endl;
    return 1;
  }
  std::string hostname = "0";
  std::string log_level = "debug";
  std::string log_file = "";
  if (argc > 2)
    hostname = argv[2];
  if (argc > 3)
    log_level = argv[3];
  if (argc > 4)
    log_file = argv[4];

  int port = std::stoi(argv[1]);
    
  return ssp_client_opencv(port, hostname);
}
#endif


