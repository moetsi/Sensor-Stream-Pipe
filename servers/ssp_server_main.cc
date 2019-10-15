//
// Created by amourao on 26-06-2019.
//

#include <unistd.h>

#include "../utils/logger.h"

#include <ctime>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <thread>
#include <time.h>

#include <yaml-cpp/yaml.h>
#include <zmq.hpp>

#include "../encoders/libav_encoder.h"
#include "../encoders/null_encoder.h"
#include "../encoders/zdepth_encoder.h"
#include "../readers/video_file_reader.h"

#ifdef SSP_WITH_NVPIPE_SUPPORT
#include "../encoders/nv_encoder.h"
#endif

#ifdef SSP_WITH_KINECT_SUPPORT
#include "../clients/ssp_coordinator_types.h"
#include "../readers/kinect_reader.h"
#include "../utils/kinect_utils.h"
#endif

std::mutex mutex_;
std::condition_variable cond_var_;
zmq::context_t context_(1);
bool ready = false;
bool leave = false;

int send_frames(std::string &yaml_config, std::string &broker_host) {
  zmq::socket_t socket(context_, ZMQ_PUSH);

  YAML::Node codec_parameters = YAML::LoadFile(yaml_config);

  YAML::Node general_parameters = codec_parameters["general"];
  SetupLogging(general_parameters);

  std::string host = codec_parameters["general"]["host"].as<std::string>();
  unsigned int port = codec_parameters["general"]["port"].as<unsigned int>();

  IReader *reader = nullptr;

  std::string reader_type =
      general_parameters["frame_source"]["type"].as<std::string>();
  if (reader_type == "frames") {
    reader =
        new ImageReader(general_parameters["frame_source"]["parameters"]["path"]
                            .as<std::string>());
  } else if (reader_type == "video") {
    std::string path = general_parameters["frame_source"]["parameters"]["path"]
                           .as<std::string>();
    if (general_parameters["frame_source"]["parameters"]["streams"]
            .IsDefined()) {
      std::vector<unsigned int> streams =
          general_parameters["frame_source"]["parameters"]["streams"]
              .as<std::vector<unsigned int>>();
      reader = new VideoFileReader(path, streams);
    } else {
      reader = new VideoFileReader(path);
    }

  } else if (reader_type == "kinect") {
#ifdef SSP_WITH_KINECT_SUPPORT
    ExtendedAzureConfig c = BuildKinectConfigFromYAML(
        general_parameters["frame_source"]["parameters"]);
    reader = new KinectReader(0, c);
#else
    exit(1);
#endif
  } else {
    spdlog::error("Unknown reader type: \"{}\". Supported types are "
                  "\"frames\", \"video\" and \"kinect\"",
                  reader_type);
    exit(1);
  }

  std::unordered_map<unsigned int, IEncoder *> encoders;

  std::vector<unsigned int> types = reader->GetType();

  for (unsigned int type : types) {
    YAML::Node v = codec_parameters["video_encoder"][type];
    std::string encoder_type = v["type"].as<std::string>();
    IEncoder *fe = nullptr;
    if (encoder_type == "libav")
      fe = new LibAvEncoder(v, reader->GetFps());
    else if (encoder_type == "nvenc") {
#ifdef SSP_WITH_NVPIPE_SUPPORT
      fe = new NvEncoder(v, reader->GetFps());
#else
      spdlog::error("SSP compiled without \"nvenc\" reader support. Set to "
                    "SSP_WITH_NVPIPE_SUPPORT=ON when configuring with cmake");
      exit(1);
#endif
    } else if (encoder_type == "zdepth")
      fe = new ZDepthEncoder(reader->GetFps());
    else if (encoder_type == "null")
      fe = new NullEncoder(reader->GetFps());
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

  socket.connect("tcp://" + broker_host);

  int linger = 0;
  socket.setsockopt(ZMQ_LINGER, &linger, sizeof(linger));

  unsigned int fps = reader->GetFps();

  while (!leave) {

    while (ready && !leave) {

      uint64_t sleep_time = (1000 / fps) - processing_time;

      if (sleep_time > 1)
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));

      start_frame_time = CurrentTimeMs();

      if (sent_frames == 0) {
        last_time = CurrentTimeMs();
        start_time = last_time;
      }

      std::vector<FrameStruct> v;
      std::vector<FrameStruct *> vO;

      while (v.empty()) {
        std::vector<FrameStruct *> frameStruct = reader->GetCurrentFrame();
        for (FrameStruct *frameStruct : frameStruct) {

          IEncoder *frameEncoder = encoders[frameStruct->frame_type];

          frameEncoder->AddFrameStruct(frameStruct);
          if (frameEncoder->HasNextPacket()) {
            FrameStruct *f = frameEncoder->CurrentFrameEncoded();
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
        std::string short_message = CerealStructToString(v);

        std::string message = v.front().stream_id + " " + short_message;

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

        spdlog::debug(
            "Message sent, took {} ms; packet size {}; avg {} fps; {} "
            "Mbps; {} Mbps expected",
            diff_time, message.size(), avg_fps,
            8 * (sent_mbytes / (CurrentTimeMs() - start_time)),
            8 * (sent_mbytes * reader->GetFps() / (sent_frames * 1000)));

        for (unsigned int i = 0; i < v.size(); i++) {
          FrameStruct f = v.at(i);
          f.frame.clear();
          spdlog::debug("\t{};{};{} sent", f.device_id, f.sensor_id,
                        f.frame_id);
          vO.at(i)->frame.clear();
          delete vO.at(i);
        }
      }
    }
  }
  socket.close();
  delete reader;
  for (auto const &x : encoders)
    delete x.second;
  return 0;
}

int main(int argc, char *argv[]) {

  spdlog::set_level(spdlog::level::debug);

  srand(time(NULL) * getpid());

  av_log_set_level(AV_LOG_QUIET);

  std::string host = "127.0.0.1";
  int port = 9999;

  if (argc < 2) {
    std::cerr << "Usage: ssp_server <parameters_file>" << std::endl;
    return 1;
  }

  std::string yaml_config_file = argv[1];

  zmq::socket_t coor_socket(context_, ZMQ_REQ);
  coor_socket.connect("tcp://" + host + ":" + std::to_string(port));

  std::string connect_msg = "1 " + std::to_string(FRAME_SOURCE_ANY);
  zmq::message_t conn_request(connect_msg.c_str(), connect_msg.size());

  int SIZE = 256 * 256;
  zmq::message_t in_request(SIZE);

  int i = 0;

  coor_socket.send(conn_request);
  coor_socket.recv(&in_request);
  coor_socket.send(conn_request);

  std::string connect_msg_rsp((char *)in_request.data(), in_request.size());
  std::string msg_type = connect_msg_rsp.substr(0, 1);
  std::string broker_host =
      connect_msg_rsp.substr(2, connect_msg_rsp.size() - 2);

  std::thread sender(send_frames, std::ref(yaml_config_file),
                     std::ref(broker_host));
  while (!leave) {
    coor_socket.recv(&in_request);
    std::string msg_rsp((char *)in_request.data(), in_request.size());

    coor_socket.send(conn_request);
    {
      std::lock_guard<std::mutex> lk(mutex_);
      ready = true;
      std::cout << "started" << std::endl;
    }
    cond_var_.notify_one();

    coor_socket.recv(&in_request);
    coor_socket.send(conn_request);
    // nanosleep((const struct timespec[]){{1, 0}}, NULL);

    {
      std::lock_guard<std::mutex> lk(mutex_);
      ready = false;
      std::cout << "stopped" << std::endl;
    }
    cond_var_.notify_one();

    coor_socket.recv(&in_request);
    coor_socket.send(conn_request);

    {
      leave = true;
      std::cout << "ending" << std::endl;
    }
  }

  sender.join();
  coor_socket.close();
  context_.close();

  return 0;
}
