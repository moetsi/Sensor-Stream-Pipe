//
// Created by amourao on 26-06-2019.
//

#include <chrono>
#include <iostream>
#include <unistd.h>

#include <k4a/k4a.h>
#include <zmq.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

#include "../utils/logger.h"

#include "../clients/ssp_coordinator_types.h"
#include "../readers/network_reader.h"
#include "../utils/kinect_utils.h"

void PrintBodyInformation(k4abt_body_t body) {
  std::cout << "Body ID: " << body.id << std::endl;
  for (int i = 0; i < (int)K4ABT_JOINT_COUNT; i++) {
    k4a_float3_t position = body.skeleton.joints[i].position;
    k4a_quaternion_t orientation = body.skeleton.joints[i].orientation;
    printf("Joint[%d]: Position[mm] ( %f, %f, %f ); Orientation ( %f, %f, %f, "
           "%f) \n",
           i, position.v[0], position.v[1], position.v[2], orientation.v[0],
           orientation.v[1], orientation.v[2], orientation.v[3]);
  }
}

void PrintBodyIndexMapMiddleLine(k4a::image body_index_map) {
  uint8_t *body_index_map_buffer = body_index_map.get_buffer();

  assert(body_index_map.get_stride_bytes() ==
         body_index_map.get_width_pixels());

  int middle_line_num = body_index_map.get_height_pixels() / 2;
  body_index_map_buffer = body_index_map_buffer +
                          middle_line_num * body_index_map.get_width_pixels();

  std::cout << "BodyIndexMap at Line " << middle_line_num << ":" << std::endl;
  for (int i = 0; i < body_index_map.get_width_pixels(); i++) {
    std::cout << (int)*body_index_map_buffer << ", ";
    body_index_map_buffer++;
  }
  std::cout << std::endl;
}

std::mutex mutex_;
std::condition_variable cond_var_;
zmq::context_t context_(1);
bool ready = false;
bool leave = false;
NetworkReader *reader;

int processor_communicator(zmq::context_t &context) {
  zmq::socket_t out_socket = zmq::socket_t(context, ZMQ_STREAM);
  std::string id = out_socket.getsockopt<std::string>(ZMQ_ROUTING_ID);
  out_socket.bind("tcp://*:" + std::to_string(10001));

  int SIZE = 256;
  zmq::message_t in_request(SIZE);

  zmq::message_t idrequest;

  int num_bodies = 0;
  std::string message =
      id + std::to_string(num_bodies) + "  bodies are detected!\n";

  while (1) {
    out_socket.recv(&idrequest);
    while (!out_socket.recv(&in_request)) {
    };
    std::cout << in_request.str() << std::endl;
    zmq::message_t request(message.size());
    memcpy(request.data(), message.c_str(), message.size());
    out_socket.send(idrequest, ZMQ_SNDMORE);
    out_socket.send(request, ZMQ_SNDMORE);
    std::cout << request.str() << std::endl;
  }
}

int worker() {

  spdlog::set_level(spdlog::level::debug);
  av_log_set_level(AV_LOG_QUIET);

  srand(time(NULL) * getpid());

  zmq::context_t context(1);
  // std::thread processor_thread(processor_communicator, std::ref(context));

  try {
    reader->init();

    k4a::calibration sensor_calibration;
    bool calibration_set = false;
    k4abt::tracker tracker;

    std::unordered_map<std::string, IDecoder *> decoders;

    while (reader->HasNextFrame()) {
      reader->NextFrame();
      std::vector<FrameStruct> f_list = reader->GetCurrentFrame();
      for (FrameStruct f : f_list) {
        std::string decoder_id = f.stream_id + std::to_string(f.sensor_id);

        if (f.camera_calibration_data.type == 0 && calibration_set == false) {

          const k4a_depth_mode_t d = static_cast<const k4a_depth_mode_t>(
              f.camera_calibration_data.extra_data[0]);
          const k4a_color_resolution_t r =
              static_cast<const k4a_color_resolution_t>(
                  f.camera_calibration_data.extra_data[1]);

          sensor_calibration = k4a::calibration::get_from_raw(
              reinterpret_cast<char *>(&f.camera_calibration_data.data[0]),
              f.camera_calibration_data.data.size(), d, r);

          calibration_set = true;

          tracker = k4abt::tracker::create(sensor_calibration);
        }
      }

      k4a::capture sensor_capture = k4a::capture::create();

      try {
        FrameStructToK4A(f_list, sensor_capture, decoders);

        if (!tracker.enqueue_capture(sensor_capture)) {
          // It should never hit timeout when K4A_WAIT_INFINITE is set.
          spdlog::error(
              "Error adding capture to tracker process queue timeout!");
          exit(1);
        }

        k4abt::frame body_frame = tracker.pop_result();

        if (body_frame != nullptr) {
          size_t num_bodies = body_frame.get_num_bodies();
          spdlog::info("{} bodies are detected!", num_bodies);

          for (size_t i = 0; i < num_bodies; i++) {
            k4abt_body_t body = body_frame.get_body(i);
            PrintBodyInformation(body);
          }

          k4a::image body_index_map = body_frame.get_body_index_map();
          if (body_index_map != nullptr) {
            PrintBodyIndexMapMiddleLine(body_index_map);
          } else {
            spdlog::error("Failed to generate bodyindex map!");
          }
        } else {
          spdlog::error("Pop body frame result time out!!");
          break;
        }
      } catch (std::exception &e) {
        spdlog::error(e.what());
      }
    }

  } catch (std::exception &e) {
    spdlog::error(e.what());
  }
  // processor_thread.join();
  return 0;
}

int main(int argc, char *argv[]) {
  spdlog::set_level(spdlog::level::debug);

  srand(time(NULL) * getpid());

  av_log_set_level(AV_LOG_QUIET);

  if (argc < 2) {
    std::cerr << "Usage: ssp_processor_k4a <port> (<log level>) (<log file>)"
              << std::endl;
    return 1;
  }
  std::string log_level = "debug";
  std::string log_file = "";

  std::string host = "127.0.0.1";
  int port = std::stoi(argv[1]);

  if (argc > 2)
    log_level = argv[2];
  if (argc > 3)
    log_file = argv[3];

  reader = new NetworkReader(port);

  std::string coor_host = "127.0.0.1";
  int coor_port = 9999;

  std::string coor_host_port = coor_host + ":" + std::to_string(coor_port);

  std::string error_msg;
  int error = 1;

  int SIZE = 256 * 256;
  zmq::message_t in_request(SIZE);
  std::string processor_id = RandomString(16);

  std::string yaml_config_file = argv[1];

  zmq::socket_t coor_socket(context_, ZMQ_REQ);

  std::string connect_msg =
      std::string(1, char(SSP_MESSAGE_CONNECT)) +
      std::string(1, char(SSP_CONNECTION_TYPE_PROCESSOR)) + host + ":" +
      std::to_string(port) + " " + processor_id + " " +
      std::string(1, char(SSP_FRAME_SOURCE_CAMERA)) + " " +
      std::string(1, char(SSP_EXCHANGE_DATA_TYPE_VECTOR_SKELETON)) + " ";
  zmq::message_t conn_request(connect_msg.c_str(), connect_msg.size());
  zmq::message_t dummy_request(std::string(1, char(SSP_MESSAGE_DUMMY)).c_str(),
                               1);

  spdlog::info("Connecting to coordinator at " + coor_host_port);

  coor_socket.connect("tcp://" + coor_host_port);

  coor_socket.send(conn_request);
  spdlog::info("Waiting to coordinator");
  coor_socket.recv(&in_request);
  spdlog::info("Coordinator responded");
  coor_socket.send(dummy_request);

  FrameSourceType type;
  std::string metadata;

  spdlog::info("Connected to coordinator " + coor_host_port);

  std::string connect_msg_rsp((char *)in_request.data(), in_request.size());

  spdlog::info("Coordinator answer " + connect_msg_rsp);

  std::thread worker_thread(worker);
  while (!leave) {
    spdlog::info("Waiting for request");
    coor_socket.recv(&in_request);
    std::string msg_rsp((char *)in_request.data(), in_request.size());
    char msg_type = msg_rsp.substr(0, 1).c_str()[0];

    /*
     * enum MsgType {
  SSP_MESSAGE_CONNECT = 0,
  SSP_MESSAGE_START,
  SSP_MESSAGE_STOP,
  SSP_MESSAGE_REG_FS,
  SSP_MESSAGE_REG_P,
  SSP_MESSAGE_REG_CON,
  SSP_MESSAGE_QUE_FS,
  SSP_MESSAGE_QUE_P,
  SSP_MESSAGE_QUE_CON,
  SSP_MESSAGE_CON_FS,
  SSP_MESSAGE_CON_P,
  SSP_MESSAGE_DATA,
  SSP_MESSAGE_OK,
  SSP_MESSAGE_ERROR
      };
     */
    switch (msg_type) {
    case SSP_MESSAGE_CONNECT: {
      char conn_type = msg_rsp.substr(1, 1).c_str()[0];
      std::string data = msg_rsp.substr(2, msg_rsp.size() - 2);
      std::string delimitor = " ";
      std::vector<std::string> sdata = SplitString(data, delimitor);
      std::string host = sdata.at(0);
      std::string id = sdata.at(1);

      reader->SetFilter(id);

      dummy_request =
          zmq::message_t(std::string(1, char(SSP_MESSAGE_DUMMY)).c_str(), 1);
      coor_socket.send(dummy_request);
      break;
    }
    case SSP_MESSAGE_DISCONNECT: {
      std::string dummy_filter = "STOP ";
      reader->SetFilter(dummy_filter);

      dummy_request =
          zmq::message_t(std::string(1, char(SSP_MESSAGE_DUMMY)).c_str(), 1);
      coor_socket.send(dummy_request);
      break;
    }
    default: {
      spdlog::info("Invalid " + std::to_string(msg_type) + " request.");
      dummy_request =
          zmq::message_t(std::string(1, char(SSP_MESSAGE_DUMMY)).c_str(), 1);
      coor_socket.send(dummy_request);
      break;
    }
    }
  }

  worker_thread.join();
  context_.close();

  return 0;
}