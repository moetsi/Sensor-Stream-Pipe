//
// Created by amourao on 26-06-2019.
//

#include "ssp_processor_k4a.h"

SSPProcK4A::SSPProcK4A(std::string host_port) {
  host_port_ = host_port;
  reader_ = std::make_shared<NetworkReader>(host_port);
}

void SSPProcK4A::Start() {
  worker_thread_ = std::thread(SSPProcK4A::Worker, std::ref(reader_));
}

void SSPProcK4A::Stop() { worker_thread_.join(); }

void SSPProcK4A::AddFilter(std::string filter) { reader_->AddFilter(filter); }
void SSPProcK4A::ClearFilter() { reader_->ClearFilter(); }
