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
#include "ssp_server.h"
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

    std::unique_lock<std::mutex> lk(mutex_);
    cond_var_.wait(lk, [] { return ready; });

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

  std::string host_port = host + ":" + std::to_string(port);

  if (argc < 2) {
    std::cerr << "Usage: ssp_server <parameters_file>" << std::endl;
    return 1;
  }
  SSPServer ssp;
  std::string error_msg;
  int error = 1;

  int SIZE = 256 * 256;
  zmq::message_t in_request(SIZE);
  int i = 0;

  std::string yaml_config_file = argv[1];

  zmq::socket_t coor_socket(context_, ZMQ_REQ);

  std::string connect_msg =
      std::string(1, char(SSP_MESSAGE_CONNECT)) +
      std::string(1, char(SSP_CONNECTION_TYPE_FRAMESOURCE)) + host + ":" +
      std::to_string(port) + " " + ssp.id + " " +
      std::string(1, char(SSP_FRAME_SOURCE_CAMERA)) + " ";
  zmq::message_t conn_request(connect_msg.c_str(), connect_msg.size());
  zmq::message_t dummy_request(std::string(1, char(SSP_MESSAGE_DUMMY)).c_str(),
                               1);

  spdlog::info("Connecting to coordinator at " + host_port);

  coor_socket.connect("tcp://" + host_port);

  coor_socket.send(conn_request);
  spdlog::info("Waiting to coordinator");
  coor_socket.recv(&in_request);
  spdlog::info("Coordinator responded");
  coor_socket.send(dummy_request);

  FrameSourceType type;
  std::string metadata;

  error = ssp.ConnectCoordinator(type, metadata, error_msg);

  if (error != 0) {
    spdlog::error("Error " + std::to_string(error) +
                  " connecting to coordinator \"" + error_msg + "\"");
  }

  spdlog::info("Connected to coordinator " + host_port);

  std::string connect_msg_rsp((char *)in_request.data(), in_request.size());

  spdlog::info("Coordinator answer " + connect_msg_rsp);

  char msg_type = connect_msg_rsp.substr(0, 1).c_str()[0];
  std::string broker_host =
      connect_msg_rsp.substr(2, connect_msg_rsp.size() - 2);

  spdlog::info("Connecting to broker at \"" + broker_host + "\"");
  std::thread sender(send_frames, std::ref(yaml_config_file),
                     std::ref(broker_host));
  error = ssp.ConnectBroker(broker_host, error_msg);

  if (error != 0) {
    spdlog::error("Error " + std::to_string(error) +
                  " connecting to broker \"" + error_msg + "\"");
  }

  while (!leave) {
    spdlog::info("Waiting for request");
    coor_socket.recv(&in_request);
    std::string msg_rsp((char *)in_request.data(), in_request.size());
    msg_type = msg_rsp.substr(0, 1).c_str()[0];

    /*
     * enum MsgType {
  SSP_MESSAGE_CONNECT = 0,
  SSP_MESSAGE_START,
  SSP_MESSAGE_STOP,
  SSP_MESSAGE_REG_FS,
  SSP_MESSAGE_REG_P,
  SSP_MESSAGE_REG_CON,
  SSP_MESSAGE_QUE_FS,
  SSP_MESSAGE_QUE_P,
  SSP_MESSAGE_QUE_CON,
  SSP_MESSAGE_CON_FS,
  SSP_MESSAGE_CON_P,
  SSP_MESSAGE_DATA,
  SSP_MESSAGE_OK,
  SSP_MESSAGE_ERROR
      };
     */
    switch (msg_type) {
    case SSP_MESSAGE_START: {
      error = ssp.Start(error_msg);
      if (error != 0) {
        spdlog::error("SSP_MESSAGE_START Error " + std::to_string(error) +
                      "\": " + error_msg + "\"");
      } else {
        spdlog::info("SSP_MESSAGE_START request");
        std::lock_guard<std::mutex> lk(mutex_);
        ready = true;
        cond_var_.notify_one();
      }
      coor_socket.send(dummy_request);
      break;
    }
    case SSP_MESSAGE_STOP: {
      error = ssp.Stop(error_msg);
      if (error != 0) {
        spdlog::error("SSP_MESSAGE_STOP Error " + std::to_string(error) +
                      "\": " + error_msg + "\"");
      } else {
        spdlog::info("SSP_MESSAGE_STOP request");
        ready = false;
      }
      coor_socket.send(dummy_request);
      break;
    }
    case SSP_MESSAGE_EXIT: {
      spdlog::info("SSP_MESSAGE_EXIT request");
      leave = true;
      if (ready == false) {
        std::lock_guard<std::mutex> lk(mutex_);
        ready = true;
        cond_var_.notify_one();
      }

      coor_socket.send(dummy_request);
      break;
    }
    default: {
      spdlog::info("Invalid " + std::to_string(msg_type) + " request.");
      coor_socket.send(dummy_request);
      break;
    }
    }
  }

  sender.join();
  coor_socket.close();
  context_.close();

  return 0;
}
