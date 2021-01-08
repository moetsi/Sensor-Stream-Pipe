//
// Created by amourao on 26-06-2019.
//

#ifdef _WIN32
#include <io.h>
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "../utils/logger.h"

#include <ctime>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <thread>

#include <yaml-cpp/yaml.h>
#include <zmq.hpp>

#include "../encoders/libav_encoder.h"
#include "../encoders/null_encoder.h"
#include "../encoders/zdepth_encoder.h"
#include "../readers/video_file_reader.h"
#include "../readers/multi_image_reader.h"

#ifdef SSP_WITH_NVPIPE_SUPPORT
#include "../encoders/nv_encoder.h"
#endif

#ifdef SSP_WITH_KINECT_SUPPORT
#include "../readers/kinect_reader.h"
#include "../utils/kinect_utils.h"
#endif

int main(int argc, char *argv[]) {

  spdlog::set_level(spdlog::level::debug);

  srand(time(NULL) * getpid());

  try {
    av_log_set_level(AV_LOG_QUIET);

    if (argc < 2) {
      std::cerr << "Usage: ssp_server <parameters_file>" << std::endl;
      return 1;
    }

    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_PUSH);

    std::string codec_parameters_file = std::string(argv[1]);

    YAML::Node codec_parameters = YAML::LoadFile(codec_parameters_file);

    YAML::Node general_parameters = codec_parameters["general"];
    SetupLogging(general_parameters);

    std::string host = codec_parameters["general"]["host"].as<std::string>();
    unsigned int port = codec_parameters["general"]["port"].as<unsigned int>();

    std::unique_ptr<IReader> reader = nullptr;

    std::string reader_type =
        general_parameters["frame_source"]["type"].as<std::string>();
    if (reader_type == "frames") {
      if (general_parameters["frame_source"]["parameters"]["path"].IsSequence())
        reader = std::unique_ptr<MultiImageReader>(new MultiImageReader(
            general_parameters["frame_source"]["parameters"]["path"]
                .as<std::vector<std::string>>()));
      else
        reader = std::unique_ptr<ImageReader>(new ImageReader(
            general_parameters["frame_source"]["parameters"]["path"]
                .as<std::string>()));

    } else if (reader_type == "video") {
      std::string path =
          general_parameters["frame_source"]["parameters"]["path"]
              .as<std::string>();
      if (general_parameters["frame_source"]["parameters"]["streams"]
              .IsDefined()) {
        std::vector<unsigned int> streams =
            general_parameters["frame_source"]["parameters"]["streams"]
                .as<std::vector<unsigned int>>();
        reader = std::unique_ptr<VideoFileReader>(
            new VideoFileReader(path, streams));
      } else {
        reader = std::unique_ptr<VideoFileReader>(new VideoFileReader(path));
      }

    } else if (reader_type == "kinect") {
#ifdef SSP_WITH_KINECT_SUPPORT
      ExtendedAzureConfig c = BuildKinectConfigFromYAML(
          general_parameters["frame_source"]["parameters"]);
      reader = std::unique_ptr<KinectReader>(new KinectReader(0, c));
#else
      exit(1);
#endif
    } else {
      spdlog::error("Unknown reader type: \"{}\". Supported types are "
                    "\"frames\", \"video\" and \"kinect\"",
                    reader_type);
      exit(1);
    }

    std::unordered_map<unsigned int, std::shared_ptr<IEncoder>> encoders;

    std::vector<unsigned int> types = reader->GetType();

    for (unsigned int type : types) {
      YAML::Node v = codec_parameters["video_encoder"][type];
      std::string encoder_type = v["type"].as<std::string>();
      std::shared_ptr<IEncoder> fe = nullptr;
      if (encoder_type == "libav")
        fe = std::shared_ptr<LibAvEncoder>(
            new LibAvEncoder(v, reader->GetFps()));
      else if (encoder_type == "nvenc") {
#ifdef SSP_WITH_NVPIPE_SUPPORT
        fe = std::shared_ptr<NvEncoder>(new NvEncoder(v, reader->GetFps()));
#else
        spdlog::error("SSP compiled without \"nvenc\" reader support. Set to "
                      "SSP_WITH_NVPIPE_SUPPORT=ON when configuring with cmake");
        exit(1);
#endif
      } else if (encoder_type == "zdepth")
        fe =
            std::shared_ptr<ZDepthEncoder>(new ZDepthEncoder(reader->GetFps()));
      else if (encoder_type == "null")
        fe = std::shared_ptr<NullEncoder>(new NullEncoder(reader->GetFps()));
      else {
        spdlog::error("Unknown encoder type: \"{}\". Supported types are "
                      "\"libav\", \"nvenc\", \"zdepth\" and \"null\"",
                      encoder_type);
        exit(1);
      }
      encoders[type] = fe;
    }

    uint64_t last_time = CurrentTimeMs();
    uint64_t start_time = last_time;
    uint64_t start_frame_time = last_time;
    uint64_t sent_frames = 0;
    uint64_t processing_time = 0;

    double sent_mbytes = 0;

    double sent_latency = 0;

    socket.connect("tcp://" + host + ":" + std::to_string(port));

    unsigned int fps = reader->GetFps();

    while (1) {

      uint64_t sleep_time = (1000 / fps) - processing_time;

      if (sleep_time > 1)
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));

      start_frame_time = CurrentTimeMs();

      if (sent_frames == 0) {
        last_time = CurrentTimeMs();
        start_time = last_time;
      }

      std::vector<FrameStruct> v;
      std::vector<std::shared_ptr<FrameStruct>> vO;

      while (v.empty()) {
        std::vector<std::shared_ptr<FrameStruct>> frameStruct =
            reader->GetCurrentFrame();
        for (std::shared_ptr<FrameStruct> frameStruct : frameStruct) {

          std::shared_ptr<IEncoder> frameEncoder =
              encoders[frameStruct->frame_type];

          frameEncoder->AddFrameStruct(frameStruct);
          if (frameEncoder->HasNextPacket()) {
            std::shared_ptr<FrameStruct> f =
                frameEncoder->CurrentFrameEncoded();
            vO.push_back(f);
            v.push_back(*f);
            frameEncoder->NextPacket();
          }
        }
        if (reader->HasNextFrame())
          reader->NextFrame();
        else {
          reader->Reset();
        }
      }

      if (!v.empty()) {
        std::string message = CerealStructToString(v);

        zmq::message_t request(message.size());
        memcpy(request.data(), message.c_str(), message.size());
        socket.send(request);
        sent_frames += 1;
        sent_mbytes += message.size() / 1000.0;

        uint64_t diff_time = CurrentTimeMs() - last_time;

        double diff_start_time = (CurrentTimeMs() - start_time);
        int64_t avg_fps;
        if (diff_start_time == 0)
          avg_fps = -1;
        else {
          double avg_time_per_frame_sent_ms =
              diff_start_time / (double)sent_frames;
          avg_fps = 1000 / avg_time_per_frame_sent_ms;
        }

        last_time = CurrentTimeMs();
        processing_time = last_time - start_frame_time;

        sent_latency += diff_time;

        spdlog::debug(
            "Message sent, took {} ms (avg. {}); packet size {}; avg {} fps; "
            "{} "
            "Mbps; {} Mbps expected",
            diff_time, sent_latency / sent_frames, message.size(), avg_fps,
            8 * (sent_mbytes / (CurrentTimeMs() - start_time)),
            8 * (sent_mbytes * reader->GetFps() / (sent_frames * 1000)));

        for (unsigned int i = 0; i < v.size(); i++) {
          FrameStruct f = v.at(i);
          f.frame.clear();
          spdlog::debug("\t{};{};{} sent", f.device_id, f.sensor_id,
                        f.frame_id);
          vO.at(i)->frame.clear();
          vO.at(i) = nullptr;
        }
      }
    }
  } catch (YAML::Exception &e) {
    spdlog::error("Error on the YAML configuration file");
    spdlog::error(e.what());
  } catch (std::exception &e) {
    spdlog::error("General Error");
    spdlog::error(e.what());
  }

  return 0;
}
