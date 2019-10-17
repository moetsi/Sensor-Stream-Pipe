//
// Created by amourao on 08/10/19.
//

#include <string>
#include <unordered_map>
#include <vector>

#include "../utils/utils.h"
#include "../utils/logger.h"
#include "ssp_coordinator_types.h"

#pragma once

class SSPCoordinator {
private:
  std::unordered_map<std::string, ProcessorInstance> processor_instances_;
  std::unordered_map<std::string, FrameServerInstance> frameserver_instances_;
  std::unordered_map<std::string, BrokerInstance> broker_instances_;

  std::unordered_map<std::string, std::string> current_connections_id_mapping_;

  std::unordered_map<std::string, FrameServerProcessorConnection>
      current_connections_;



public:
  BrokerInstance broker_;

  SSPCoordinator();

  ~SSPCoordinator();

  int RegisterFrameSource(const std::string &host, const std::string &id,
                          const std::string &zmq_id,
                          const FrameSourceType &type,
                          const std::string &metadata, std::string &error);

  int RegisterBroker(const std::string &host, const std::string &id,
                     const std::string &zmq_id, std::string &error);

  int RegisterProcessor(const std::string &host, const std::string &id,
                        const std::string &zmq_id, const FrameSourceType &type,
                        const ExchangeDataType &out_type,
                        const std::string &metadata, std::string &error);

  int Connect(const std::string &id_fs, const std::string &id_proc,
              FrameServerProcessorConnection &connection, std::string &error);

  int Disconnect(const std::string &id_out, std::string &error);

  int GetBrokers(std::vector<std::pair<std::string, std::string>> &results,
                 std::string &error);

  int GetFrameSources(
      std::vector<std::pair<std::string, FrameSourceType>> &results,
      std::string &error);

  int GetProcessors(
      std::vector<
          std::pair<std::string, std::pair<FrameSourceType, ExchangeDataType>>>
          &results,
      std::string &error);

  int GetConnections(
      std::vector<
          std::pair<std::string, std::pair<FrameSourceType, ExchangeDataType>>>
      &results,
      std::string &error);

  int GetProcessorInfo(const std::string &id, ProcessorInstance &metadata,
                       std::string &error);

  int GetFrameServerInfo(const std::string &id, FrameServerInstance &metadata,
                         std::string &error);

  int GetConnectionInfo(const std::string &id,
                        FrameServerProcessorConnection &metadata,
                        std::string &error);

  int SetMetadata(const std::string &id, const std::string &metadata,
                  std::string &error);

  int Start(const std::string &id, std::string &error);

  int Stop(const std::string &id, std::string &error);
};
