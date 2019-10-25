//
// Created by amourao on 25-10-2019.
//
#include "ssp_processor_opencv.h"

SSPProcOpenCV::SSPProcOpenCV(std::string host_port) {
  host_port_ = host_port;
  reader_ = std::make_shared<NetworkReader>(host_port);
}

void SSPProcOpenCV::Start() {
  worker_thread_ = std::thread(SSPProcOpenCV::Worker, std::ref(reader_));
}

void SSPProcOpenCV::Stop() { worker_thread_.join(); }

void SSPProcOpenCV::AddFilter(std::string filter) { reader_->AddFilter(filter); }
void SSPProcOpenCV::ClearFilter() { reader_->ClearFilter(); }

