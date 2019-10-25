//
// Created by amourao on 25-10-2019.
//
#include "ssp_processor_template.h"

SSPProcTemplate::SSPProcTemplate(std::string host_port) {
  host_port_ = host_port;
  reader_ = std::make_shared<NetworkReader>(host_port);
}

void SSPProcTemplate::Start() {
  worker_thread_ = std::thread(SSPProcTemplate::Worker, std::ref(reader_));
}

void SSPProcTemplate::Stop() { worker_thread_.join(); }

void SSPProcTemplate::AddFilter(std::string filter) { reader_->AddFilter(filter); }
void SSPProcTemplate::ClearFilter() { reader_->ClearFilter(); }

