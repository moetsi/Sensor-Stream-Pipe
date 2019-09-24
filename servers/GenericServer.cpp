//
// Created by amourao on 26-06-2019.
//

#include <unistd.h>

#include <k4a/k4a.h>

#include <ctime>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <thread>

#include <yaml-cpp/yaml.h>
#include <zmq.hpp>

#include "../encoders/LibAvEncoder.h"
#include "../encoders/NullEncoder.h"
#include "../encoders/NvEncoder.h"
#include "../encoders/ZDepthEncoder.h"
#include "../readers/KinectReader.h"
#include "../readers/VideoFileReader.h"

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
    setupLogging(general_parameters);

    std::string host = codec_parameters["general"]["host"].as<std::string>();
    uint port = codec_parameters["general"]["port"].as<uint>();

    IReader *reader = nullptr;

    std::string reader_type =
        general_parameters["frame_source"]["type"].as<std::string>();
    if (reader_type == "frames") {
      reader = new ImageReader(
          general_parameters["frame_source"]["parameters"]["path"]
              .as<std::string>());
    } else if (reader_type == "video") {
      std::string path =
          general_parameters["frame_source"]["parameters"]["path"]
              .as<std::string>();
      if (general_parameters["frame_source"]["parameters"]["streams"]
              .IsDefined()) {
        std::vector<uint> streams =
            general_parameters["frame_source"]["parameters"]["streams"]
                .as<std::vector<uint>>();
        reader = new VideoFileReader(path, streams);
      } else {
        reader = new VideoFileReader(path);
      }
    } else if (reader_type == "kinect") {
      ExtendedAzureConfig c = buildKinectConfigFromYAML(
          general_parameters["frame_source"]["parameters"]);
      reader = new KinectReader(0, c);
    } else {
      spdlog::error("Unknown reader type: \"{}\". Supported types are "
                    "\"frames\", \"video\" and \"kinect\"",
                    reader_type);
      exit(1);
    }

    std::unordered_map<uint, IEncoder *> encoders;

    std::vector<uint> types = reader->getType();

    for (uint type : types) {
      YAML::Node v = codec_parameters["video_encoder"][type];
      std::string encoder_type = v["type"].as<std::string>();
      IEncoder *fe = nullptr;
      if (encoder_type == "libav")
        fe = new LibAvEncoder(v, reader->getFps());
      else if (encoder_type == "nvenc")
        fe = new NvEncoder(v, reader->getFps());
      else if (encoder_type == "zdepth")
        fe = new ZDepthEncoder(reader->getFps());
      else if (encoder_type == "null")
        fe = new NullEncoder(reader->getFps());
      else {
        spdlog::error("Unknown encoder type: \"{}\". Supported types are "
                      "\"libav\", \"nvenc\", \"zdepth\" and \"null\"",
                      encoder_type);
        exit(1);
      }
      encoders[type] = fe;
    }

    uint64_t last_time = currentTimeMs();
    uint64_t start_time = last_time;
    uint64_t start_frame_time = last_time;
    uint64_t sent_frames = 0;
    uint64_t processing_time = 0;

    double sent_mbytes = 0;

    socket.connect("tcp://" + host + ":" + std::to_string(port));

    uint fps = reader->getFps();

    while (1) {

      uint64_t sleep_time = (1000 / fps) - processing_time;

      if (sleep_time > 1)
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));

      start_frame_time = currentTimeMs();

      if (sent_frames == 0) {
        last_time = currentTimeMs();
        start_time = last_time;
      }

      std::vector<FrameStruct> v;
      std::vector<FrameStruct *> vO;

      while (v.empty()) {
        std::vector<FrameStruct *> frameStruct = reader->currentFrame();
        for (FrameStruct *frameStruct : frameStruct) {
          IEncoder *frameEncoder = encoders[frameStruct->frameType];

          frameEncoder->addFrameStruct(frameStruct);
          if (frameEncoder->hasNextPacket()) {
            FrameStruct *f = frameEncoder->currentFrameEncoded();
            vO.push_back(f);
            v.push_back(*f);
            frameEncoder->nextPacket();
          }
        }
        if (reader->hasNextFrame())
          reader->nextFrame();
        else {
          reader->reset();
        }
      }

      if (!v.empty()) {
        std::string message = cerealStructToString(v);

        zmq::message_t request(message.size());
        memcpy(request.data(), message.c_str(), message.size());
        socket.send(request);
        sent_frames += 1;
        sent_mbytes += message.size() / 1000.0;

        uint64_t diff_time = currentTimeMs() - last_time;

        double diff_start_time = (currentTimeMs() - start_time);
        int64_t avg_fps;
        if (diff_start_time == 0)
          avg_fps = -1;
        else {
          double avg_time_per_frame_sent_ms =
              diff_start_time / (double)sent_frames;
          avg_fps = 1000 / avg_time_per_frame_sent_ms;
        }

        last_time = currentTimeMs();
        processing_time = last_time - start_frame_time;

        spdlog::debug(
            "Message sent, took {} ms; packet size {}; avg {} fps; {} "
            "Mbps; {} Mbps expected",
            diff_time, request.size(), avg_fps,
            8 * (sent_mbytes / (currentTimeMs() - start_time)),
            8 * (sent_mbytes * reader->getFps() / (sent_frames * 1000)));

        for (uint i = 0; i < v.size(); i++) {
          FrameStruct f = v.at(i);
          f.frame.clear();
          spdlog::debug("\t{};{};{} sent", f.deviceId, f.sensorId, f.frameId);
          vO.at(i)->frame.clear();
          delete vO.at(i);
        }
      }
    }
    delete reader;
    for (auto const &x : encoders)
      delete x.second;
  } catch (YAML::Exception &e) {
    spdlog::error("Error on the YAML configuration file");
    spdlog::error(e.what());
  } catch (std::exception &e) {
    spdlog::error("General Error");
    spdlog::error(e.what());
  }

  return 0;
}
