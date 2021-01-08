//
// Created by amourao on 27-09-2019.
//

#include "network_reader.h"
#include "../utils/video_utils.h"

NetworkReader::NetworkReader(int port) {
  last_time_ = CurrentTimeMs();
  start_time_ = last_time_;
  rec_frames_ = 0;
  rec_mbytes_ = 0;
  port_ = port;
}

void NetworkReader::init() {
  context_ = std::unique_ptr<zmq::context_t>(new zmq::context_t(1));
  socket_ = std::unique_ptr<zmq::socket_t>(new zmq::socket_t(*context_, ZMQ_PULL));
#ifdef SSP_WITH_ZMQ_POLLING
  poller_.add(zmq::socket_ref(zmq::from_handle, socket_.get()->handle()),
      zmq::event_flags::pollin);

#endif
  socket_->bind("tcp://*:" + std::to_string(port_));
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

void NetworkReader::NextFrame() {
  zmq::message_t request;

  socket_->recv(&request);

  if (rec_frames_ == 0) {
    last_time_ = CurrentTimeMs();
    start_time_ = last_time_;
  }

  rec_frames_ += 1;
  uint64_t diff_time = CurrentTimeMs() - last_time_;
  double diff_start_time = (CurrentTimeMs() - start_time_) / (double)rec_frames_;
  int64_t avg_fps;
  if (diff_start_time == 0)
    avg_fps = -1;
  else
    avg_fps = 1000 / diff_start_time;

  last_time_ = CurrentTimeMs();

  std::string result =
      std::string(static_cast<char *>(request.data()), request.size());

  std::vector<FrameStruct> f_list =
      ParseCerealStructFromString<std::vector<FrameStruct>>(result);

  rec_mbytes_ += request.size() / 1000;

  for (unsigned int i = 0; i < f_list.size(); i++) {
    f_list.at(i).timestamps.push_back(CurrentTimeMs());
  }

  current_frame_counter_++;
  current_frame_internal_ = f_list;

  spdlog::debug(
      "Message received, took {} ms; packet size {}; avg {} fps; {} avg "
      "Mbps; latency: {} ms",
      diff_time, request.size(), avg_fps,
      8 * (rec_mbytes_ / (CurrentTimeMs() - start_time_)),
      (f_list.front().timestamps.back() - f_list.front().timestamps.at(1)));
  for (FrameStruct f : f_list) {
    std::string decoder_id = f.stream_id + std::to_string(f.sensor_id);
    rec_mbytes_per_stream_[decoder_id] += f.frame.size() / 1000;
    spdlog::debug("\t{};{};{} {} avg Mbps; latency: {} ms", f.device_id,
                  f.sensor_id, f.frame_id,
                  8 * (rec_mbytes_per_stream_[decoder_id] /
                       (CurrentTimeMs() - start_time_)),
                  (f.timestamps.back() - f.timestamps.at(1)));
  }

}

std::vector<FrameStruct> NetworkReader::GetCurrentFrame() {
  return current_frame_internal_;
}

unsigned int NetworkReader::GetCurrentFrameId() { return current_frame_counter_; }
