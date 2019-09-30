//
// Created by amourao on 27-09-2019.
//

#include "network_reader.h"
#include "../utils/video_utils.h"

NetworkReader::NetworkReader(int _port) {
  port = _port;
}

void NetworkReader::init() {

  context = new zmq::context_t(1);
  socket = new zmq::socket_t(*context, ZMQ_PULL);

  socket->bind("tcp://*:" + std::to_string(port));
}

NetworkReader::~NetworkReader() {
  socket->close();
  delete socket;
}

bool NetworkReader::hasNextFrame() { return true; }

void NetworkReader::nextFrame() {
  zmq::message_t request;

  socket->recv(&request);

  if (rec_frames == 0) {
    last_time = currentTimeMs();
    start_time = last_time;
  }

  rec_frames += 1;
  uint64_t diff_time = currentTimeMs() - last_time;
  double diff_start_time = (currentTimeMs() - start_time) / (double)rec_frames;
  int64_t avg_fps;
  if (diff_start_time == 0)
    avg_fps = -1;
  else
    avg_fps = 1000 / diff_start_time;

  last_time = currentTimeMs();

  std::string result =
      std::string(static_cast<char *>(request.data()), request.size());

  std::vector<FrameStruct> f_list =
      parseCerealStructFromString<std::vector<FrameStruct>>(result);

  rec_mbytes += request.size() / 1000;

  for (uint i = 0; i < f_list.size(); i++) {
    f_list.at(i).timestamps.push_back(currentTimeMs());
  }

  currentFrameCounter++;
  currentFrameInternal = f_list;

  spdlog::debug(
      "Message received, took {} ms; packet size {}; avg {} fps; {} avg "
      "Mbps; latency: {} ms",
      diff_time, request.size(), avg_fps,
      8 * (rec_mbytes / (currentTimeMs() - start_time)),
      (f_list.front().timestamps.back() - f_list.front().timestamps.at(1)));
  for (FrameStruct f : f_list) {
    std::string decoder_id = f.streamId + std::to_string(f.sensorId);
    rec_mbytes_per_stream[decoder_id] += f.frame.size() / 1000;
    spdlog::debug("\t{};{};{} {} avg Mbps; latency: {} ms", f.deviceId,
                  f.sensorId, f.frameId,
                  8 * (rec_mbytes_per_stream[decoder_id] /
                       (currentTimeMs() - start_time)),
                  (f.timestamps.back() - f.timestamps.at(1)));
  }

}

std::vector<FrameStruct> NetworkReader::currentFrame() {
  return currentFrameInternal;
}

uint NetworkReader::currentFrameId() { return currentFrameCounter; }
