//
// Created by amourao on 08/10/19.
//

#include "ssp_coordinator.h"

SSPCoordinator::SSPCoordinator() {}
SSPCoordinator::~SSPCoordinator() {}

int SSPCoordinator::RegisterFrameSource(const std::string &host,
                                        const std::string &id,
                                        const std::string &zmq_id,
                                        const FrameSourceType &type,
                                        const std::string &metadata,
                                        std::string &error) {
  if (frameserver_instances_.find(id) != frameserver_instances_.end() ||
      processor_instances_.find(id) != processor_instances_.end() ||
      broker_instances_.find(id) != broker_instances_.end()) {
    error = "Id already exists: " + id;
    spdlog::error(error);

    return 1;
  }

  if (broker_.status == SSP_EXEC_STATUS_UNINIT) {
    error = "No broker registered when registering fs: " + id;
    spdlog::error(error);
    return 2;
  }

  FrameServerInstance fsi;
  fsi.metadata = metadata;
  fsi.source_type = type;
  fsi.host = host;
  fsi.status = SSP_EXEC_STATUS_STOPPED;
  fsi.id = id;
  fsi.zmq_id = zmq_id;
  frameserver_instances_[id] = fsi;

  spdlog::info("FrameSource registered: " + id);
  return 0;
}

int SSPCoordinator::RegisterProcessor(
    const std::string &host, const std::string &id, const std::string &zmq_id,
    const FrameSourceType &type,
                                      const ExchangeDataType &out_type,
                                      const std::string &metadata,
                                      std::string &error) {
  if (frameserver_instances_.find(id) != frameserver_instances_.end() ||
      processor_instances_.find(id) != processor_instances_.end() ||
      broker_instances_.find(id) != broker_instances_.end()) {
    error = "Id already exists: " + id;
    return 1;
  }
  ProcessorInstance pi;
  pi.metadata = metadata;
  pi.source_type = type;
  pi.data_type = out_type;
  pi.host = host;
  pi.status = SSP_EXEC_STATUS_STOPPED;
  pi.id = id;
  pi.zmq_id = zmq_id;
  processor_instances_[id] = pi;

  spdlog::info("Processor registered: " + id);
  return 0;
}

int SSPCoordinator::RegisterBroker(const std::string &host,
                                   const std::string &id,
                                   const std::string &zmq_id,
                                   std::string &error) {
  if (frameserver_instances_.find(id) != frameserver_instances_.end() ||
      processor_instances_.find(id) != processor_instances_.end() ||
      broker_instances_.find(id) != broker_instances_.end()) {
    error = "Id already exists: " + id;
    return 1;
  }
  BrokerInstance bi;
  bi.host = host;
  bi.status = SSP_EXEC_STATUS_RUNNING;
  bi.id = id;
  bi.zmq_id = zmq_id;
  broker_instances_[id] = bi;

  broker_ = bi;
  spdlog::info("Broker registered: " + id);
  return 0;
}

int SSPCoordinator::Connect(const std::string &id_fs,
                            const std::string &id_proc,
                            FrameServerProcessorConnection &connection,
                            std::string &error) {

  std::string fspc_id = id_fs + "_" + id_proc;

  if (current_connections_id_mapping_.find(fspc_id) !=
      current_connections_id_mapping_.end()) {
    error = "Connection already exists: " + fspc_id;
    spdlog::error(error);
    return 1;
  }

  if (processor_instances_.find(id_proc) == processor_instances_.end()) {
    error = "Processor id does not exist: " + id_proc;
    spdlog::error(error);
    return 2;
  }

  if (frameserver_instances_.find(id_fs) == frameserver_instances_.end()) {
    error = "FrameServer id does not exist: " + id_fs;
    spdlog::error(error);
    return 3;
  }

  ProcessorInstance pi = processor_instances_[id_proc];
  FrameServerInstance fsi = frameserver_instances_[id_fs];

  if (pi.source_type != SSP_FRAME_SOURCE_ANY &&
      pi.source_type != fsi.source_type) {
    error = "Incompatible datatypes between processor and frame server; PI: " +
            std::to_string(pi.source_type) +
            " FSI: " + std::to_string(fsi.source_type);
    spdlog::error(error);
    return 4;
  }

  connection.id = RandomString(16);
  connection.status = SSP_EXEC_STATUS_STOPPED;
  connection.frameserver = fsi;
  connection.processor = pi;

  current_connections_id_mapping_[fspc_id] = connection.id;
  current_connections_[connection.id] = connection;

  spdlog::info("Connection registered: " + fspc_id);
  return 0;
}
int SSPCoordinator::Disconnect(const std::string &id_out, std::string &error) {
  if (current_connections_.find(id_out) == current_connections_.end()) {
    error = "Connection does not exist: " + id_out;
    spdlog::error(error);
    return 1;
  }

  FrameServerProcessorConnection connection = current_connections_[id_out];

  std::string id_fs = connection.frameserver.id;
  std::string id_proc = connection.processor.id;
  std::string fspc_id = id_fs + "_" + id_proc;
  current_connections_.erase(id_out);
  current_connections_id_mapping_.erase(fspc_id);

  spdlog::info("Connection disconnected: " + fspc_id);
  return 0;
}
int SSPCoordinator::GetFrameSources(
    std::vector<std::pair<std::string, FrameSourceType>> &results,
    std::string &error) {

  for (auto &fsi : frameserver_instances_) {
    std::pair<std::string, FrameSourceType> pair;
    pair.first = fsi.first;
    pair.second = fsi.second.source_type;
    results.push_back(pair);
  }

  return 0;
}

int SSPCoordinator::GetProcessors(
    std::vector<
        std::pair<std::string, std::pair<FrameSourceType, ExchangeDataType>>>
        &results,
    std::string &error) {

  for (auto &ps : processor_instances_) {
    std::pair<std::string, std::pair<FrameSourceType, ExchangeDataType>> pair;
    pair.first = ps.first;
    std::pair<FrameSourceType, ExchangeDataType> pair1;
    pair1.first = ps.second.source_type;
    pair1.second = ps.second.data_type;
    pair.second = pair1;
    results.push_back(pair);
  }

  return 0;
}

int SSPCoordinator::GetBrokers(
    std::vector<std::pair<std::string, std::string>> &results,
    std::string &error) {

  for (auto &ps : broker_instances_) {
    std::pair<std::string, std::string> pair;
    pair.first = ps.first;
    pair.second = ps.second.host;
    results.push_back(pair);
  }

  return 0;
}

int SSPCoordinator::GetProcessorInfo(const std::string &id,
                                     ProcessorInstance &metadata,
                                     std::string &error) {

  if (processor_instances_.find(id) == processor_instances_.end()) {
    error = "Processor id does not exist: " + id;
    spdlog::error(error);
    return 1;
  }

  metadata = processor_instances_[id];

  return 0;
}

int SSPCoordinator::GetFrameServerInfo(const std::string &id,
                                       FrameServerInstance &metadata,
                                       std::string &error) {

  if (frameserver_instances_.find(id) == frameserver_instances_.end()) {
    error = "FrameServer id does not exist: " + id;
    spdlog::error(error);
    return 1;
  }

  metadata = frameserver_instances_[id];

  return 0;
}

int SSPCoordinator::GetConnectionInfo(const std::string &id,
                                      FrameServerProcessorConnection &metadata,
                                      std::string &error) {

  if (current_connections_.find(id) == current_connections_.end()) {
    error = "Connection id does not exist: " + id;
    spdlog::error(error);
    return 1;
  }

  metadata = current_connections_[id];

  return 0;
}

int SSPCoordinator::SetMetadata(const std::string &id,
                                const std::string &metadata,
                                std::string &error) {

  return 0;
}

int SSPCoordinator::Start(const std::string &id, std::string &error) {
  if (current_connections_.find(id) == current_connections_.end()) {
    error = "Connection does not exist: " + id;
    spdlog::error(error);
    return 1;
  }

  FrameServerProcessorConnection fspc = current_connections_[id];

  if (fspc.status == SSP_EXEC_STATUS_RUNNING) {
    error = "Connection already running: " + id;
    spdlog::error(error);
    return 2;
  }

  current_connections_[id].status = SSP_EXEC_STATUS_RUNNING;

  spdlog::info("Connection started: " + fspc.id);
  return 0;
}

int SSPCoordinator::Stop(const std::string &id, std::string &error) {
  if (current_connections_.find(id) == current_connections_.end()) {
    error = "Connection does not exist: " + id;
    spdlog::error(error);
    return 1;
  }

  FrameServerProcessorConnection fspc = current_connections_[id];

  if (fspc.status != SSP_EXEC_STATUS_RUNNING) {
    error = "Connection is not running: " + id;
    spdlog::error(error);
    return 2;
  }

  current_connections_[id].status = SSP_EXEC_STATUS_STOPPED;
  spdlog::info("Connection stopped: " + fspc.id);
  return 0;
}
