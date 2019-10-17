//
// Created by amourao on 10/10/19.
//

#pragma once

#include "../clients/ssp_coordinator_types.h"
#include "../utils/utils.h"

class SSPServer {
private:
  BrokerInstance broker_;
  BrokerInstance coordinator_;

  ExecStatus status_;

public:
  std::string id;

  SSPServer();
  ~SSPServer();

  int ConnectBroker(const std::string &host,
                    std::string &error);

  int ConnectCoordinator(const FrameSourceType &type,
                         const std::string &metadata, std::string &error);

  int GetBroker(BrokerInstance &connection, std::string &error);

  int GetCoordinator(BrokerInstance &connection, std::string &error);

  int Start(std::string &error);

  int Stop(std::string &error);

  int HandleMessage(std::string &message);
};
