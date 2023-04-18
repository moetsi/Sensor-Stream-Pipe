/**
 * \file network_reader.cc @brief Network reader
 */
// Created by amourao on 27-09-2019.

#include "network_reader.h"
#include "../utils/video_utils.h"

namespace moetsi::ssp {

NetworkReader::NetworkReader(int port) {
  last_time_ = CurrentTimeNs();
  start_time_ = last_time_;
  rec_frames_ = 0;
  rec_mbytes_ = 0;
  port_ = port;
}

void NetworkReader::init() {

  context_ = std::unique_ptr<zmq::context_t>(new zmq::context_t(1));
  socket_ = std::unique_ptr<zmq::socket_t>(new zmq::socket_t(*context_, ZMQ_PULL));
  // Do not accumulate packets if no client is connected
  socket_->set(zmq::sockopt::immediate, true);
  // Do not keep packets if there is network congestion
  socket_->set(zmq::sockopt::conflate, true);

#ifdef SSP_WITH_ZMQ_POLLING
  poller_.add(zmq::socket_ref(zmq::from_handle, socket_.get()->handle()),
      zmq::event_flags::pollin);

#endif

  //context = zmq_ctx_new();
  //responder = zmq_socket(context, ZMQ_PULL);
  //auto bind_str = "tcp://*:" + std::to_string(port_);               
  //auto rc = zmq_bind(responder, bind_str.c_str());
  //std::cerr << "rc = " << rc << std::endl << std::flush;

  socket_->bind("tcp://*:" + std::to_string(port_));
  

}

void NetworkReader::init(std::string hostname) {

  context_ = std::unique_ptr<zmq::context_t>(new zmq::context_t(1));
  socket_ = std::unique_ptr<zmq::socket_t>(new zmq::socket_t(*context_, ZMQ_PULL));
  // Do not accumulate packets if no client is connected
  socket_->set(zmq::sockopt::immediate, true);
  // Do not keep packets if there is network congestion
  socket_->set(zmq::sockopt::conflate, true);

#ifdef SSP_WITH_ZMQ_POLLING
  poller_.add(zmq::socket_ref(zmq::from_handle, socket_.get()->handle()),
      zmq::event_flags::pollin);

#endif

  //context = zmq_ctx_new();
  //responder = zmq_socket(context, ZMQ_PULL);
  //auto bind_str = "tcp://*:" + std::to_string(port_);               
  //auto rc = zmq_bind(responder, bind_str.c_str());
  //std::cerr << "rc = " << rc << std::endl << std::flush;
  hostname_ = hostname;

  socket_->connect("tcp://" + hostname_ + ":" + std::to_string(port_));
  
}

NetworkReader::~NetworkReader() {
  socket_->close();
}

bool NetworkReader::HasNextFrame() {
#ifdef SSP_WITH_ZMQ_POLLING
    std::vector<zmq::poller_event<>> poller_ev(1);
    const auto recv = poller_.wait_all(poller_ev, std::chrono::milliseconds(POLL_TIMEOUT_MS));
    if (recv <= 0) return false;
    else return true;
#else
    return true;
#endif
}

unsigned long elapsed(unsigned long start, unsigned long end)
{
  if (end < start)
    return start - end;
  else
    return end - start;
}

void NetworkReader::NextFrame() {

// std::cerr << __FILE__ << ":" << __LINE__<< std::endl << std::flush;
  zmq::message_t request;
  socket_->recv(&request);
//  std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush; 
  //uint32_t l32 = 0;
  //auto rc = zmq_recv(responder, &l32, 4, 0);
  //std::cerr << "l32 = " << l32 << std::endl;
  //std::cerr << __FILE__ << ":" << __LINE__<< std::endl << std::flush;
  //int sockopt = 0;
  //size_t sockopt_len = sizeof(int);
  //auto rcso = zmq_getsockopt(responder, ZMQ_RCVMORE, &sockopt, &sockopt_len);
  //std::vector<char> buffer_;
  //buffer_.resize(l32);
  //auto rc2 = zmq_recv(responder, &buffer_[0], buffer_.size(), 0);

  if (rec_frames_ == 0) {
    last_time_ = CurrentTimeNs();
    start_time_ = last_time_;
  }

  rec_frames_ += 1;
  uint64_t diff_time = CurrentTimeNs() - last_time_;
  double diff_start_time = (CurrentTimeNs() - start_time_) / (double)rec_frames_;
  int64_t avg_fps;
  if (diff_start_time == 0)
    avg_fps = -1;
  else
    avg_fps = 1000000000ULL / diff_start_time;

  last_time_ = CurrentTimeNs();

  std::string result =
      std::string(static_cast<char *>(request.data()), request.size());
    
  current_serialized_message_ = std::make_shared<std::string>(result);

  //std::string result =
  //    std::string(static_cast<char *>(&buffer_[0]), buffer_.size());

  std::vector<FrameStruct> f_list =
      ParseCerealStructFromString<std::vector<FrameStruct>>(result);

  rec_mbytes_ += request.size() / 1000;
  // rec_mbytes_ += buffer_.size() / 1000;

  for (unsigned int i = 0; i < f_list.size(); i++) {
    f_list.at(i).timestamps.push_back(CurrentTimeNs());
  }

  current_frame_counter_++;
  current_frame_internal_ = f_list;

  spdlog::debug(
      "Message received, took {} ns; packet size {}; avg {} fps; {:3.2f} avg "
      "Mbps; latency: {} ns",
      diff_time, 
      request.size(), 
    //  buffer_.size(),
      avg_fps,
      8 * (1000000ULL * rec_mbytes_ / (CurrentTimeNs() - start_time_)),
      elapsed(f_list.front().timestamps.back(), f_list.front().timestamps.at(1)));
  for (FrameStruct f : f_list) {
    std::string decoder_id = f.stream_id + std::to_string(f.sensor_id);
    rec_mbytes_per_stream_[decoder_id] += f.frame.size() / 1000;
    spdlog::debug("\t{};{};{} {:3.2f} avg Mbps; latency: {} ns", f.device_id,
                  f.sensor_id, f.frame_id,
                  8 * (1000000ULL * rec_mbytes_per_stream_[decoder_id] /
                       (CurrentTimeNs() - start_time_)),
                  elapsed(f.timestamps.back(), f.timestamps.at(1)));
  }

}

std::shared_ptr<std::string> NetworkReader::GetCurrentSerializedMessage() {
  return current_serialized_message_;
}

std::vector<FrameStruct> NetworkReader::GetCurrentFrame() {
  return current_frame_internal_;
}

unsigned int NetworkReader::GetCurrentFrameId() { return current_frame_counter_; }

} // namespace moetsi::ssp
