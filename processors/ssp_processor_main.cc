#ifdef SSP_WITH_KINECT_SUPPORT
#include "ssp_processor_pointcloud.h"
#endif
#ifdef SSP_WITH_K4A_BODYTRACK
#include "ssp_processor_k4a.h"
#endif

#include "ssp_processor_opencv.h"
#include "ssp_processor_template.h"

int main(int argc, char *argv[]) {
  spdlog::set_level(spdlog::level::debug);

  srand(time(NULL) * getpid());

  av_log_set_level(AV_LOG_QUIET);

  if (argc < 3) {
    std::cerr
        << "Usage: ssp_processor <processor_type> <coor_host_port> (<args>)"
        << std::endl;
    return 1;
  }

  // TODO: allow these as args
  std::string log_level = "debug";
  std::string log_file = "";

  std::string processor_type = argv[1];
  std::string coor_host_port = argv[2];
  std::string args = "";

  // TODO: clean
  if (processor_type != "k4a" && processor_type != "opencv" &&
      processor_type != "pointcloud" && processor_type != "template") {
    spdlog::error("Unknown processor_type: {}", processor_type);
    exit(0);
  }

#ifndef SSP_WITH_K4A_BODYTRACK
  if (processor_type == "k4a")
    spdlog::error("SSP compiled without \"bodytrack\" support. Set to "
                  "SSP_WITH_K4A_BODYTRACK=ON when configuring with cmake");
  exit(1);
#endif

#ifndef SSP_WITH_KINECT_SUPPORT
  if (processor_type == "pointcloud")
    spdlog::error("SSP compiled without \"pointcloud\" support. Set to "
                  "SSP_WITH_KINECT_SUPPORT=ON when configuring with cmake");
  exit(1);
#endif

  if (argc > 3)
    args = argv[3];

  std::string error_msg;
  int error = 1;

  int SIZE = 256 * 256;
  zmq::message_t in_request(SIZE);
  std::string processor_id = RandomString(16);

  std::string yaml_config_file = argv[1];

  zmq::context_t context;
  bool leave = false;

  zmq::socket_t coor_socket(context, ZMQ_REQ);

  std::string connect_msg =
      std::string(1, char(SSP_MESSAGE_CONNECT)) +
      std::string(1, char(SSP_CONNECTION_TYPE_PROCESSOR)) + coor_host_port +
      " " + processor_id + " " +
      std::string(1, char(SSP_EXCHANGE_DATA_TYPE_VECTOR_FRAME_STRUCT)) + " " +
      std::string(1, char(SSP_EXCHANGE_DATA_TYPE_VECTOR_SKELETON)) + " ";
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

  ExchangeDataType type;
  std::string metadata;

  spdlog::info("Connected to coordinator " + coor_host_port);

  std::string connect_msg_rsp((char *)in_request.data(), in_request.size());

  spdlog::info("Coordinator answer " + connect_msg_rsp);

  std::string broker_host_port =
      connect_msg_rsp.substr(2, connect_msg_rsp.size() - 2);

  ISSPProcessor *instance;

  if (processor_type == "opencv") {
    instance = new SSPProcOpenCV(broker_host_port);
  } else if (processor_type == "template") {
    instance = new SSPProcTemplate(broker_host_port);
#ifdef SSP_WITH_K4A_BODYTRACK
  } else if (processor_type == "k4a") {
    instance = new SSPProcK4A(broker_host_port);
#else
    spdlog::error("SSP compiled without \"bodytrack\" support. Set to "
                  "SSP_WITH_K4A_BODYTRACK=ON when configuring with cmake");
    exit(1);
#endif
#ifdef SSP_WITH_KINECT_SUPPORT
  } else if (processor_type == "pointcloud") {
    instance = new SSPProcPointcloud(broker_host_port, args);
#else
    spdlog::error("SSP compiled without \"pointcloud\" support. Set to "
                  "SSP_WITH_KINECT_SUPPORT=ON when configuring with cmake");
    exit(1);
#endif
  } else {
    exit(0);
  }

  instance->Start();

  while (!leave) {
    spdlog::info("Waiting for request");
    coor_socket.recv(&in_request);
    std::string msg_rsp((char *)in_request.data(), in_request.size());
    char msg_type = msg_rsp.substr(0, 1).c_str()[0];

    switch (msg_type) {
    case SSP_MESSAGE_CONNECT: {
      char conn_type = msg_rsp.substr(1, 1).c_str()[0];
      std::string data = msg_rsp.substr(3, msg_rsp.size() - 3);
      std::string delimitor = " ";
      std::vector<std::string> sdata = SplitString(data, delimitor);
      std::string host = sdata.at(0);
      std::string id = sdata.at(1);

      instance->AddFilter(id);

      dummy_request =
          zmq::message_t(std::string(1, char(SSP_MESSAGE_DUMMY)).c_str(), 1);
      coor_socket.send(dummy_request);
      break;
    }
    case SSP_MESSAGE_DISCONNECT: {
      instance->ClearFilter();

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

  instance->Stop();
  context.close();

  return 0;
}