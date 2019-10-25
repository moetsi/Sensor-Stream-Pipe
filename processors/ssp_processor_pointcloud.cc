//
// Created by amourao on 25-10-2019.
//
#include "ssp_processor_pointcloud.h"

SSPProcPointcloud::SSPProcPointcloud(std::string host_port,
                               std::string write_pattern) {
  host_port_ = host_port;
  write_pattern_ = write_pattern;
  reader_ = std::make_shared<NetworkReader>(host_port);
}

void SSPProcPointcloud::Start() {
  worker_thread_ = std::thread(SSPProcPointcloud::Worker, std::ref(reader_),
                               std::ref(write_pattern_));
}

void SSPProcPointcloud::Stop() { worker_thread_.join(); }

void SSPProcPointcloud::AddFilter(std::string filter) {
  reader_->AddFilter(filter);
}
void SSPProcPointcloud::ClearFilter() { reader_->ClearFilter(); }
