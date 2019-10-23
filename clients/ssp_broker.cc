//
// Created by amourao on 26-06-2019.
//

#include <chrono>
#include <iostream>
#include <unistd.h>
#include <zmq.hpp>

#include "../utils/logger.h"
#include "../utils/utils.h"
#include "ssp_coordinator_types.h"

zmq::context_t context_(1);

bool leave;
int Sender(int port_in, int port_out) {

  spdlog::set_level(spdlog::level::debug);

  srand(time(NULL) * getpid());

  try {
    zmq::socket_t *in_socket = new zmq::socket_t(context_, ZMQ_PULL);
    in_socket->bind("tcp://*:" + std::to_string(port_in));

    zmq::socket_t *out_socket = new zmq::socket_t(context_, ZMQ_PUB);
    out_socket->bind("tcp://*:" + std::to_string(port_out));

    // Can I use ZeroMQ to interact with normal sockets and, for example, be
    // able to ping www.google.com from a ZeroMQ socket?
    //
    // You can in version 4.x use the ZMQ_STREAM socket to speak raw TCP.

    zmq::proxy(*in_socket, *out_socket, nullptr);
    /*
    zmq::message_t request;

    while (1) {
      in_socket->recv(&request);
      std::string stream_id =
          std::string(static_cast<char *>(request.data()), 16);
      spdlog::debug("Message received {}", stream_id);
      out_socket->send(request);
      spdlog::debug("Message sent");
    }
    */
  } catch (std::exception &e) {
    spdlog::error(e.what());
  }

  return 0;
}

int main(int argc, char *argv[]) {

  spdlog::set_level(spdlog::level::debug);

  srand(time(NULL) * getpid());

  try {

    if (argc < 2) {
      std::cerr << "Usage: ssp_broker <coordinator_host_port> (<log level>) "
                   "(<log file>)"
                << std::endl;
      return 1;
    }
    std::string log_level = "debug";
    std::string log_file = "";

    std::string id = RandomString(16);

    std::string coor_host_port = argv[1];

    if (argc > 2)
      log_level = argv[2];
    if (argc > 3)
      log_file = argv[3];

    zmq::socket_t coor_socket(context_, ZMQ_REQ);

    zmq::message_t in_request;

    std::string connect_msg = std::string(1, char(SSP_MESSAGE_CONNECT)) +
                              std::string(1, char(SSP_CONNECTION_TYPE_BROKER)) +
                              " " + id;

    zmq::message_t conn_request(connect_msg.c_str(), connect_msg.size());
    zmq::message_t dummy_request(
        std::string(1, char(SSP_MESSAGE_DUMMY)).c_str(), 1);

    spdlog::info("Connecting to coordinator at " + coor_host_port);

    coor_socket.connect("tcp://" + coor_host_port);

    coor_socket.send(conn_request);
    spdlog::info("Waiting to coordinator");
    coor_socket.recv(&in_request);
    spdlog::info("Coordinator responded");
    coor_socket.send(dummy_request);

    spdlog::info("Connected to coordinator " + coor_host_port);

    std::string connect_msg_rsp((char *)in_request.data(), in_request.size());

    std::string data = connect_msg_rsp.substr(2, connect_msg_rsp.size() - 2);
    std::string delimitor = " ";
    std::vector<std::string> sdata = SplitString(data, delimitor);

    int port_in = std::stoi(sdata[0]);
    int port_out = std::stoi(sdata[1]);
    ;
    std::thread sender(Sender, port_in, port_out);

    while (!leave) {
      spdlog::info("Waiting for request");
      coor_socket.recv(&in_request);
      std::string msg_rsp((char *)in_request.data(), in_request.size());
      char msg_type = msg_rsp.substr(0, 1).c_str()[0];

      switch (msg_type) {
      case SSP_MESSAGE_DISCONNECT: {
        spdlog::info("SSP_MESSAGE_DISCONNECT request");
        dummy_request =
            zmq::message_t(std::string(1, char(SSP_MESSAGE_DUMMY)).c_str(), 1);
        coor_socket.send(dummy_request);

        context_.close();
        sender.join();
        break;
      }
      case SSP_MESSAGE_EXIT: {
        spdlog::info("SSP_MESSAGE_EXIT request");
        leave = true;
        dummy_request =
            zmq::message_t(std::string(1, char(SSP_MESSAGE_DUMMY)).c_str(), 1);
        coor_socket.send(dummy_request);

        context_.close();
        sender.join();
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

  } catch (std::exception &e) {
    spdlog::error(e.what());
  }

  return 0;
}
