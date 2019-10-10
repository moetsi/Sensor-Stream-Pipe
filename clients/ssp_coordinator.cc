//
// Created by amourao on 08/10/19.
//

#include "ssp_coordinator.h"
#include "../utils/utils.h"
SSPCoordinator::SSPCoordinator() {}
SSPCoordinator::~SSPCoordinator() {}

int SSPCoordinator::RegisterFrameSource(const std::string &host,
                                        const int &port, const std::string &id,
                                        const FrameSourceType &type,
                                        const std::string &metadata,
                                        std::string &error) {
  if (frameserver_instances_.find(id) != frameserver_instances_.end() ||
      processor_instances_.find(id) != processor_instances_.end() ||
      broker_instances_.find(id) != broker_instances_.end()) {
    error = "Id already exists";
    return 1;
  }

  if (broker_.status == EXEC_STATUS_UNINIT) {
    error = "No broker registered";
    return 2;
  }

  FrameServerInstance fsi;
  fsi.metadata = metadata;
  fsi.source_type = type;
  fsi.host = host;
  fsi.port = port;
  fsi.status = EXEC_STATUS_STOPPED;
  fsi.id = id;
  frameserver_instances_[id] = fsi;
  return 0;
}

int SSPCoordinator::RegisterProcessor(const std::string &host, const int &port,
                                      const std::string &id,
                                      const FrameSourceType &type,
                                      const ExchangeDataType &out_type,
                                      const std::string &metadata,
                                      std::string &error) {
  if (frameserver_instances_.find(id) != frameserver_instances_.end() ||
      processor_instances_.find(id) != processor_instances_.end() ||
      broker_instances_.find(id) != broker_instances_.end()) {
    error = "Id already exists";
    return 1;
  }
  ProcessorInstance pi;
  pi.metadata = metadata;
  pi.source_type = type;
  pi.data_type = out_type;
  pi.host = host;
  pi.port = port;
  pi.status = EXEC_STATUS_STOPPED;
  pi.id = id;
  processor_instances_[id] = pi;
  return 0;
}

int SSPCoordinator::RegisterBroker(const std::string &host, const int &port,
                                   const std::string &id, std::string &error) {
  if (frameserver_instances_.find(id) != frameserver_instances_.end() ||
      processor_instances_.find(id) != processor_instances_.end() ||
      broker_instances_.find(id) != broker_instances_.end()) {
    error = "Id already exists";
    return 1;
  }
  BrokerInstance bi;
  bi.host = host;
  bi.port = port;
  bi.status = EXEC_STATUS_RUNNING;
  bi.id = id;
  broker_instances_[id] = bi;

  broker_ = bi;
  return 0;
}

int SSPCoordinator::Connect(const std::string &id_fs,
                            const std::string &id_proc,
                            FrameServerProcessorConnection &connection,
                            std::string &error) {

  std::string fspc_id = id_fs + "_" + id_proc;

  if (current_connections_id_mapping_.find(fspc_id) !=
      current_connections_id_mapping_.end()) {
    error = "Connection already exists";
    return 1;
  }

  if (processor_instances_.find(id_proc) == processor_instances_.end()) {
    error = "Processor id does not exists";
    return 2;
  }

  if (frameserver_instances_.find(id_fs) == frameserver_instances_.end()) {
    error = "FrameServer id does not exists";
    return 3;
  }

  ProcessorInstance pi = processor_instances_[id_proc];
  FrameServerInstance fsi = frameserver_instances_[id_fs];

  if (pi.source_type != FRAME_SOURCE_ANY && pi.source_type != fsi.source_type) {
    error = "Incompatible datatypes between processor and frame server";
    return 4;
  }

  connection.id = RandomString(16);
  connection.status = EXEC_STATUS_STOPPED;
  connection.frameserver = fsi;
  connection.processor = pi;

  current_connections_id_mapping_[fspc_id] = connection.id;
  current_connections_[connection.id] = connection;

  return 0;
}
int SSPCoordinator::Disconnect(const std::string &id_out, std::string &error) {
  if (current_connections_.find(id_out) == current_connections_.end()) {
    error = "Connection does not exist";
    return 1;
  }

  FrameServerProcessorConnection connection = current_connections_[id_out];

  std::string id_fs = connection.frameserver.id;
  std::string id_proc = connection.processor.id;
  std::string fspc_id = id_fs + "_" + id_proc;
  current_connections_.erase(id_out);
  current_connections_id_mapping_.erase(fspc_id);

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
    std::vector<std::pair<std::string, std::pair<std::string, int>>> &results,
    std::string &error) {

  for (auto &ps : broker_instances_) {
    std::pair<std::string, std::pair<std::string, int>> pair;
    pair.first = ps.first;
    std::pair<std::string, int> pair1;
    pair1.first = ps.second.host;
    pair1.second = ps.second.port;
    pair.second = pair1;
    results.push_back(pair);
  }

  return 0;
}

int SSPCoordinator::GetProcessorInfo(const std::string &id,
                                     ProcessorInstance &metadata,
                                     std::string &error) {

  if (processor_instances_.find(id) == processor_instances_.end()) {
    error = "Processor id does not exists";
    return 1;
  }

  metadata = processor_instances_[id];

  return 0;
}

int SSPCoordinator::GetFrameServerInfo(const std::string &id,
                                       FrameServerInstance &metadata,
                                       std::string &error) {

  if (frameserver_instances_.find(id) == frameserver_instances_.end()) {
    error = "FrameServer id does not exists";
    return 1;
  }

  metadata = frameserver_instances_[id];

  return 0;
}

int SSPCoordinator::SetMetadata(const std::string &id,
                                const std::string &metadata,
                                std::string &error) {

  return 0;
}

int SSPCoordinator::Start(const std::string &id, std::string &error) {
  if (current_connections_.find(id) == current_connections_.end()) {
    error = "Connection does not exist";
    return 1;
  }

  FrameServerProcessorConnection fspc = current_connections_[id];

  if (fspc.status != EXEC_STATUS_STOPPED) {
    error = "Connection is not stopped";
    return 2;
  }

  current_connections_[id].status = EXEC_STATUS_RUNNING;
  return 0;
}

int SSPCoordinator::Stop(const std::string &id, std::string &error) {
  if (current_connections_.find(id) == current_connections_.end()) {
    error = "Connection does not exist";
    return 1;
  }

  FrameServerProcessorConnection fspc = current_connections_[id];

  if (fspc.status != EXEC_STATUS_RUNNING) {
    error = "Connection is not running";
    return 2;
  }

  current_connections_[id].status = EXEC_STATUS_STOPPED;
  return 0;
}

int main() {
  SSPCoordinator ssp_coordinator;
  FrameServerProcessorConnection connection;
  std::string error;
  int error_int = 0;

  std::string host;
  int port;
  std::string id_b = "a";
  std::string id_fs = "b";
  std::string id_proc = "c";
  FrameSourceType fs1_type = FRAME_SOURCE_VIDEO;
  FrameSourceType fs2_type = FRAME_SOURCE_ANY;
  ExchangeDataType edt_type = EXCHANGE_DATA_TYPE_VECTOR_CV_MAT;
  std::string metadata;

  error_int = ssp_coordinator.RegisterBroker(host, port, id_b, error);
  std::cout << error_int << " " << error << std::endl;

  error_int = ssp_coordinator.RegisterProcessor(host, port, id_proc, fs2_type,
                                                edt_type, metadata, error);
  std::cout << error_int << " " << error << std::endl;

  error_int = ssp_coordinator.RegisterFrameSource(host, port, id_fs, fs1_type,
                                                  metadata, error);
  std::cout << error_int << " " << error << std::endl;

  error_int = ssp_coordinator.Connect(id_fs, id_proc, connection, error);
  std::cout << error_int << " " << error << std::endl;

  error_int = ssp_coordinator.Start(connection.id, error);
  std::cout << error_int << " " << error << std::endl;
  return 0;
}
