//
// Created by amourao on 26-06-2019.
//

#include <unistd.h>

#include <k4a/k4a.h>

#include <ctime>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <thread>

#include <yaml-cpp/yaml.h>
#include <zmq.hpp>

#include "../encoders/LibAvEncoder.h"
#include "../encoders/NullEncoder.h"
#include "../encoders/NvEncoder.h"
#include "../encoders/ZDepthEncoder.h"
#include "../readers/KinectReader.h"
#include "../structs/FrameStruct.hpp"
#include "../utils/KinectUtils.h"
#include "../utils/Utils.h"

int main(int argc, char *argv[]) {

  srand(time(NULL) * getpid());

  try {
    av_log_set_level(AV_LOG_QUIET);

    if (argc < 4) {
      std::cerr << "Usage: server <host> <port> <codec_parameters_file>"
                << std::endl;
      return 1;
    }

    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_PUSH);

    std::string host = std::string(argv[1]);
    uint port = std::stoul(argv[2]);
    std::string codec_parameters_file = std::string(argv[3]);

    YAML::Node codec_parameters = YAML::LoadFile(codec_parameters_file);

    ExtendedAzureConfig c =
        buildKinectConfigFromYAML(codec_parameters["kinect_parameters"][0]);

    IReader *reader = new KinectReader(0, c);

    // TODO: use smarter pointers
    std::unordered_map<uint, IEncoder *> encoders;

    std::vector<uint> types = reader->getType();

    for (uint type : types) {
      YAML::Node v = codec_parameters["video_encoder"][type];
      std::string encoder_type = v["type"].as<std::string>();
      IEncoder *fe;
      if (encoder_type == "libav")
        fe = new LibAvEncoder(v, reader->getFps());
      else if (encoder_type == "nvenc")
        fe = new NvEncoder(v, reader->getFps());
      else if (encoder_type == "zdepth")
        fe = new ZDepthEncoder(reader->getFps());
      else if (encoder_type == "null")
        fe = new NullEncoder(reader->getFps());
      encoders[type] = fe;
    }

    uint64_t last_time = currentTimeMs();
    uint64_t start_time = last_time;
    uint64_t start_frame_time = last_time;
    uint64_t sent_frames = 0;
    uint64_t processing_time = 0;

    double sent_mbytes = 0;

    socket.connect("tcp://" + host + ":" + std::string(argv[2]));

    while (1) {
      start_frame_time = currentTimeMs();

      if (sent_frames == 0) {
        last_time = currentTimeMs();
        start_time = last_time;
      }

      std::vector<FrameStruct> v;
      std::vector<FrameStruct *> vO;

      // TODO: document what is happening with the Encoders and Queue
      while (v.empty()) {
        std::vector<FrameStruct *> frameStruct = reader->currentFrame();
        for (FrameStruct *frameStruct : frameStruct) {
          IEncoder *frameEncoder = encoders[frameStruct->frameType];

          frameEncoder->addFrameStruct(frameStruct);
          if (frameEncoder->hasNextPacket()) {
            FrameStruct *f = frameEncoder->currentFrameEncoded();
            vO.push_back(f);
            v.push_back(*f);
            frameEncoder->nextPacket();
          }
        }
        reader->nextFrame();
      }

      if (!v.empty()) {
        std::string message = cerealStructToString(v);

        zmq::message_t request(message.size());
        memcpy(request.data(), message.c_str(), message.size());
        socket.send(request);
        sent_frames += 1;
        sent_mbytes += message.size() / 1000.0;

        uint64_t diff_time = currentTimeMs() - last_time;

        double diff_start_time = (currentTimeMs() - start_time);
        int64_t avg_fps;
        if (diff_start_time == 0)
          avg_fps = -1;
        else {
          double avg_time_per_frame_sent_ms =
              diff_start_time / (double)sent_frames;
          avg_fps = 1000 / avg_time_per_frame_sent_ms;
        }

        last_time = currentTimeMs();
        processing_time = last_time - start_frame_time;

        std::cout << "Took " << diff_time << " ms; size " << message.size()
                  << "; avg " << avg_fps << " fps; "
                  << 8 * (sent_mbytes / diff_start_time) << " Mbps "
                  << 8 * (sent_mbytes * reader->getFps() / (sent_frames * 1000))
                  << " Mbps expected " << std::endl;
        for (uint i = 0; i < v.size(); i++) {
          FrameStruct f = v.at(i);
          f.frame.clear();
          std::cout << "\t" << f.deviceId << ";" << f.sensorId << ";"
                    << f.frameId << " sent" << std::endl;
          delete vO.at(i);
        }
      }
    }
  } catch (std::exception &e) {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
