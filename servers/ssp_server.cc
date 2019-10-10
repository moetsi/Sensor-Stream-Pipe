//
// Created by amourao on 10/10/19.
//

#include "ssp_server.h"

SSPServer::SSPServer() {}

SSPServer::~SSPServer() {}

int SSPServer::ConnectBroker(const std::string &host, const int &port,
                             std::string &error) {
  // broker, coordinator = socket.sendConnectTo(host, port)
  // broker_ = broker
  // coordinator_ = coordinator
  return 0;
}

int SSPServer::ConnectCoordinator(std::string &error) {
  // status = socket.sendConnectTo(broker.host, broker.port)
  return 0;
}

int SSPServer::GetBroker(BrokerInstance &connection, std::string &error) {
  if (broker_.status == EXEC_STATUS_UNINIT) {
    error = "Broker not initialized";
    return 1;
  }
  connection = broker_;
  return 0;
}

int SSPServer::GetCoordinator(BrokerInstance &connection, std::string &error) {
  if (coordinator_.status == EXEC_STATUS_UNINIT) {
    error = "Cooordinator not initialized";
    return 1;
  }
  connection = coordinator_;
  return 0;
}

int SSPServer::Start(std::string &error) {
  if (status_ == EXEC_STATUS_RUNNING) {
    error = "Server not initialized";
    return 1;
  }

  if (broker_.status != EXEC_STATUS_RUNNING) {
    error = "Broker not running";
    return 2;
  }

  if (status_ == EXEC_STATUS_RUNNING) {
    error = "Server already running";
    return 2;
  }

  status_ = EXEC_STATUS_RUNNING;
  return 0;
}

int SSPServer::Stop(std::string &error) {
  if (status_ == EXEC_STATUS_RUNNING) {
    error = "Server not initialized";
    return 1;
  }

  if (broker_.status == EXEC_STATUS_UNINIT) {
    error = "Broker not initialized";
    return 1;
  }

  if (status_ != EXEC_STATUS_RUNNING) {
    error = "Server not running";
    return 2;
  }

  status_ = EXEC_STATUS_STOPPED;
  return 0;
}
