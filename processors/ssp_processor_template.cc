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

std::mutex mutex_;
std::condition_variable cond_var_;
zmq::context_t context_(1);
bool ready = false;
bool leave = false;
NetworkReader *reader;

int worker() {

  spdlog::set_level(spdlog::level::debug);
  av_log_set_level(AV_LOG_QUIET);

  srand(time(NULL) * getpid());

  zmq::context_t context(1);
  // std::thread processor_thread(processor_communicator, std::ref(context));

  try {
    reader->init();

    std::unordered_map<std::string, IDecoder *> decoders;

    bool imgChanged = false;
    while (reader->HasNextFrame()) {
    }

  } catch (std::exception &e) {
    spdlog::error(e.what());
  }

  return 0;
}

int main(int argc, char *argv[]) {
  spdlog::set_level(spdlog::level::debug);

  srand(time(NULL) * getpid());

  av_log_set_level(AV_LOG_QUIET);

  if (argc < 2) {
    std::cerr << "Usage: ssp_processor_k4a <port> (<log level>) (<log file>)"
              << std::endl;
    return 1;
  }
  std::string log_level = "debug";
  std::string log_file = "";

  std::string host = "127.0.0.1";
  int port = std::stoi(argv[1]);

  if (argc > 2)
    log_level = argv[2];
  if (argc > 3)
    log_file = argv[3];

  reader = new NetworkReader(port);

  std::string coor_host = "127.0.0.1";
  int coor_port = 9999;

  std::string coor_host_port = coor_host + ":" + std::to_string(coor_port);

  std::string error_msg;
  int error = 1;

  int SIZE = 256 * 256;
  zmq::message_t in_request(SIZE);
  std::string processor_id = RandomString(16);

  std::string yaml_config_file = argv[1];

  zmq::socket_t coor_socket(context_, ZMQ_REQ);

  std::string connect_msg =
      std::string(1, char(SSP_MESSAGE_CONNECT)) +
      std::string(1, char(SSP_CONNECTION_TYPE_PROCESSOR)) + host + ":" +
      std::to_string(port) + " " + processor_id + " " +
      std::string(1, char(SSP_FRAME_SOURCE_ANY)) + " " +
      std::string(1, char(SSP_EXCHANGE_DATA_TYPE_VECTOR_CV_MAT)) + " ";
  zmq::message_t conn_request(connect_msg.c_str(), connect_msg.size());
  zmq::message_t dummy_request(std::string(1, char(SSP_MESSAGE_DUMMY)).c_str(),
                               1);

  spdlog::info("Connecting to coordinator at " + coor_host_port);

  coor_socket.connect("tcp://" + coor_host_port);

  coor_socket.send(conn_request);
  spdlog::info("Waiting to coordinator");
  coor_socket.recv(&in_request);
  spdlog::info("Coordinator responded");
  coor_socket.send(dummy_request);

  FrameSourceType type;
  std::string metadata;

  spdlog::info("Connected to coordinator " + coor_host_port);

  std::string connect_msg_rsp((char *)in_request.data(), in_request.size());

  spdlog::info("Coordinator answer " + connect_msg_rsp);

  std::thread worker_thread(worker);
  while (!leave) {
    spdlog::info("Waiting for request");
    coor_socket.recv(&in_request);
    std::string msg_rsp((char *)in_request.data(), in_request.size());
    char msg_type = msg_rsp.substr(0, 1).c_str()[0];

    switch (msg_type) {
    case SSP_MESSAGE_CONNECT: {
      char conn_type = msg_rsp.substr(1, 1).c_str()[0];
      std::string data = msg_rsp.substr(2, msg_rsp.size() - 2);
      std::string delimitor = " ";
      std::vector<std::string> sdata = SplitString(data, delimitor);
      std::string host = sdata.at(0);
      std::string id = sdata.at(1);

      reader->SetFilter(id);

      dummy_request =
          zmq::message_t(std::string(1, char(SSP_MESSAGE_DUMMY)).c_str(), 1);
      coor_socket.send(dummy_request);
      break;
    }
    case SSP_MESSAGE_DISCONNECT: {
      std::string dummy_filter = "STOP ";
      reader->ResetFilter();

      dummy_request =
          zmq::message_t(std::string(1, char(SSP_MESSAGE_DUMMY)).c_str(), 1);
      coor_socket.send(dummy_request);
      break;
    }
    default: {
      spdlog::info("Invalid " + std::to_string(msg_type) + " request.");
      dummy_request =
          zmq::message_t(std::string(1, char(SSP_MESSAGE_DUMMY)).c_str(), 1);
      coor_socket.send(dummy_request);
      break;
    }
    }
  }

  worker_thread.join();
  context_.close();

  return 0;
}
