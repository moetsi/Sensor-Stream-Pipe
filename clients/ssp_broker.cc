//
// Created by amourao on 26-06-2019.
//

#include <chrono>
#include <iostream>
#include <unistd.h>
#include <zmq.hpp>

#include "../utils/logger.h"

int main(int argc, char *argv[]) {

  spdlog::set_level(spdlog::level::debug);

  srand(time(NULL) * getpid());

  try {

    if (argc < 2) {
      std::cerr
          << "Usage: ssp_client_template <port> (<log level>) (<log file>)"
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

    zmq::context_t *context = new zmq::context_t(4);

    zmq::socket_t *in_socket = new zmq::socket_t(*context, ZMQ_PULL);
    in_socket->bind("tcp://*:" + std::to_string(port));

    zmq::socket_t *outA_socket = new zmq::socket_t(*context, ZMQ_PUB);
    outA_socket->bind("tcp://*:" + std::to_string(port + 1));

    zmq::message_t request;

    while (1) {
      in_socket->recv(&request);
      spdlog::debug("Message received");
      outA_socket->send(request);
      spdlog::debug("Message sent");
    }

  } catch (std::exception &e) {
    spdlog::error(e.what());
  }

  return 0;
}
