/**
 * \file ssp_server.cc @brief SSP, server side.
 */
// Created by amourao on 26-06-2019.

#ifdef _WIN32
#include <io.h>
#include <windows.h>
#define SSP_EXPORT __declspec(dllexport)
#else
#include <unistd.h>
#define SSP_EXPORT
#endif

#ifdef __APPLE__
#include <TargetConditionals.h>
#if TARGET_OS_IOS
#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>
#include "../readers/iphone_reader.h"
#endif
#endif

#include "../utils/logger.h"

#include <ctime>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <thread>

#include <yaml-cpp/yaml.h>
#include <zmq.hpp>

#include "../encoders/libav_encoder.h"
#include "../encoders/null_encoder.h"
#include "../encoders/zdepth_encoder.h"
#include "../readers/video_file_reader.h"
#include "../readers/multi_image_reader.h"
#include "../readers/dummy_body_reader.h"

#ifdef SSP_WITH_NVPIPE_SUPPORT
#include "../encoders/nv_encoder.h"
#endif

#ifdef SSP_WITH_KINECT_SUPPORT
#include "../readers/kinect_reader.h"
#include "../utils/kinect_utils.h"
#endif

using namespace moetsi::ssp;

extern "C" SSP_EXPORT int ssp_server(const char* filename)
{
  av_log_set_level(AV_LOG_QUIET);

  try {
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_PUSH);

    //void *context = nullptr;
    //void *requester = nullptr;
    //context = zmq_ctx_new();
    //requester = zmq_socket(context, ZMQ_PUSH);
    // Do not accumulate packets if no client is connected
    socket.set(zmq::sockopt::immediate, true);

    // Do not keep packets if there is network congestion
    // socket.set(zmq::sockopt::conflate, true);

    std::string codec_parameters_file = std::string(filename);

    YAML::Node codec_parameters = YAML::LoadFile(codec_parameters_file);

    YAML::Node general_parameters = codec_parameters["general"];
    SetupLogging(general_parameters);

    std::string host = codec_parameters["general"]["host"].as<std::string>();
    unsigned int port = codec_parameters["general"]["port"].as<unsigned int>();

    std::unique_ptr<IReader> reader = nullptr;

    std::string reader_type =
        general_parameters["frame_source"]["type"].as<std::string>();
    if (reader_type == "frames") {
      if (general_parameters["frame_source"]["parameters"]["path"].IsSequence())
        reader = std::unique_ptr<MultiImageReader>(new MultiImageReader(
            general_parameters["frame_source"]["parameters"]["path"]
                .as<std::vector<std::string>>()));
      else
        reader = std::unique_ptr<ImageReader>(new ImageReader(
            general_parameters["frame_source"]["parameters"]["path"]
                .as<std::string>()));

    } else if (reader_type == "dummybody") {
      reader = std::unique_ptr<DummyBodyReader>(new DummyBodyReader());

    } else if (reader_type == "video") {
      std::string path =
          general_parameters["frame_source"]["parameters"]["path"]
              .as<std::string>();

#if TARGET_OS_IOS
      // Find the corresponding path in the application bundle
      NSString* file_path = [NSString stringWithCString:path.c_str()
                                               encoding:[NSString defaultCStringEncoding]];
      NSString* bundle_path = [[NSBundle mainBundle] pathForResource:file_path
                                                              ofType:nil];
      if (bundle_path != nil)
        path = std::string([bundle_path UTF8String]);
#endif

        
      if (general_parameters["frame_source"]["parameters"]["streams"]
              .IsDefined()) {
        std::vector<unsigned int> streams =
            general_parameters["frame_source"]["parameters"]["streams"]
                .as<std::vector<unsigned int>>();
        reader = std::unique_ptr<VideoFileReader>(
            new VideoFileReader(path, streams));
      } else {
        reader = std::unique_ptr<VideoFileReader>(new VideoFileReader(path));
      }

    } else if (reader_type == "kinect") {
#ifdef SSP_WITH_KINECT_SUPPORT
      ExtendedAzureConfig c = BuildKinectConfigFromYAML(
          general_parameters["frame_source"]["parameters"]);
      reader = std::unique_ptr<KinectReader>(new KinectReader(0, c));
#else
      return 1;
#endif
    } else if (reader_type == "iphone") {
#if TARGET_OS_IOS
      reader = std::unique_ptr<iPhoneReader>(new iPhoneReader());
#else
      return 1;
#endif
    } else {
      spdlog::error("Unknown reader type: \"{}\". Supported types are "
                    "\"frames\", \"video\" and \"kinect\"",
                    reader_type);
      return 1;
    }

    std::unordered_map<unsigned int, std::shared_ptr<IEncoder>> encoders;

    std::vector<FrameType> types = reader->GetType();

    for (FrameType type : types) {
      YAML::Node v = codec_parameters["video_encoder"][unsigned(type)];
      std::string encoder_type = v["type"].as<std::string>();
      std::shared_ptr<IEncoder> fe = nullptr;
      if (encoder_type == "libav")
        fe = std::shared_ptr<LibAvEncoder>(
            new LibAvEncoder(v, reader->GetFps()));
      else if (encoder_type == "nvenc") {
#ifdef SSP_WITH_NVPIPE_SUPPORT
        fe = std::shared_ptr<NvEncoder>(new NvEncoder(v, reader->GetFps()));
#else
        spdlog::error("SSP compiled without \"nvenc\" reader support. Set to "
                      "SSP_WITH_NVPIPE_SUPPORT=ON when configuring with cmake");
        return 1;
#endif
      } else if (encoder_type == "zdepth")
        fe =
            std::shared_ptr<ZDepthEncoder>(new ZDepthEncoder(v, reader->GetFps()));
      else if (encoder_type == "null")
        fe = std::shared_ptr<NullEncoder>(new NullEncoder(reader->GetFps()));
      else {
        spdlog::error("Unknown encoder type: \"{}\". Supported types are "
                      "\"libav\", \"nvenc\", \"zdepth\" and \"null\"",
                      encoder_type);
        return 1;
      }

      // std::cerr << encoder_type << " " << unsigned(type) << " " << (!!fe) << std::endl << std::flush;
      encoders[unsigned(type)] = fe;
    }

    uint64_t last_time = CurrentTimeNs();
    uint64_t start_time = last_time;
    uint64_t start_frame_time = last_time;
    uint64_t sent_frames = 0;
    uint64_t processing_time = 0;

    double sent_kbytes = 0;

    double sent_latency = 0;

   socket.connect("tcp://" + host + ":" + std::to_string(port));
   // auto cstr =  "tcp://" + host + ":" + std::to_string(port);
   // auto rcc = zmq_connect(requester, cstr.c_str());

    unsigned int fps = reader->GetFps();    
    unsigned int frame_time = 1000000000ULL/fps;

    while (1) {

      if (processing_time < frame_time)
      {
        uint64_t sleep_time = frame_time - processing_time;
        std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_time));
      }

      start_frame_time = CurrentTimeNs();

      if (sent_frames == 0) {
        last_time = CurrentTimeNs();
        start_time = last_time;
      }

      std::vector<FrameStruct> v;
      std::vector<std::shared_ptr<FrameStruct>> vO;

      while (v.empty()) {
        std::vector<std::shared_ptr<FrameStruct>> frameStruct =
            reader->GetCurrentFrame();
        for (std::shared_ptr<FrameStruct> frameStruct : frameStruct) {


          std::shared_ptr<IEncoder> frameEncoder =
              encoders[unsigned(frameStruct->frame_type)];
              // std::cerr << "ft = " << unsigned(frameStruct->frame_type) << std::endl << std::endl;
          if (!!frameEncoder) {
            frameEncoder->AddFrameStruct(frameStruct);
            if (frameEncoder->HasNextPacket()) {
              std::shared_ptr<FrameStruct> f =
                  frameEncoder->CurrentFrameEncoded();
              vO.push_back(f);
              v.push_back(*f);
              frameEncoder->NextPacket();
            }
          } else {
            // std::cerr << "skip!" << std::endl << std::flush;
          }
        }
        if (reader->HasNextFrame())
          reader->NextFrame();
        else {
          reader->Reset();
        }
      }

      if (!v.empty()) {

       
        std::string message = CerealStructToString(v);

        zmq::message_t request(message.size());
        memcpy(request.data(), message.c_str(), message.size());
        socket.send(request, zmq::send_flags::none);
        //auto *buffer = &message[0];
        //auto length = message.size();
        //uint32_t l32 = length;
        //std::cerr << "l32 = " << l32 << std::endl;
        //auto rc0 = zmq_send(requester, &l32, 4, ZMQ_SNDMORE); 
        //auto rc = zmq_send(requester, buffer, length, 0);

        sent_frames += 1;
        sent_kbytes += message.size() / 1000.0;

        uint64_t diff_time = CurrentTimeNs() - last_time;

        double diff_start_time = (CurrentTimeNs() - start_time);
        int64_t avg_fps;
        if (diff_start_time == 0)
          avg_fps = -1;
        else {
          double avg_time_per_frame_sent_ms =
              diff_start_time / (double)sent_frames;
          avg_fps = 1000000000ULL / avg_time_per_frame_sent_ms;
        }

        last_time = CurrentTimeNs();
        processing_time = last_time - start_frame_time;

        sent_latency += diff_time;

        spdlog::debug(
            "Message sent, took {} ns (avg. {:3.2f}); packet size {}; avg {} fps; "
            "{:3.2f} Mbps; {:3.2f} Mbps expected",
            diff_time, sent_latency / sent_frames, message.size(), avg_fps,
            8 * (sent_kbytes * 1000000ULL / (CurrentTimeNs() - start_time)),
            8 * (sent_kbytes * reader->GetFps() / (sent_frames * 1000)));

        for (unsigned int i = 0; i < v.size(); i++) {
          FrameStruct f = v.at(i);
          f.frame.clear();
          spdlog::debug("\t{};{};{} sent", f.device_id, f.sensor_id,
                        f.frame_id);
          vO.at(i)->frame.clear();
          vO.at(i) = nullptr;
        }
      }
    }
  } catch (YAML::Exception &e) {
    spdlog::error("Error on the YAML configuration file");
    spdlog::error(e.what());
  } catch (std::exception &e) {
    spdlog::error("General Error");
    spdlog::error(e.what());
  }

  return 0;
}

#ifndef SSP_PLUGIN
int main(int argc, char *argv[]) {

  spdlog::set_level(spdlog::level::debug);

  srand(time(NULL));

  std::string filename;
#if TARGET_OS_IOS
  // Path to embedded config file
  NSString* path = [[NSBundle mainBundle] pathForResource:@"serve_ios_raw"
                                                   ofType:@"yaml"];
  if (path != nil)
    filename = std::string([path UTF8String]);

  // Launch ssp_server in a background queue
  dispatch_queue_t aQueue = dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0);
  dispatch_async(aQueue, ^{ ssp_server(filename.c_str()); });

  @autoreleasepool {
    return UIApplicationMain(argc, argv, nil, nil);
  }
#else
  if (argc < 2) {
    std::cerr << "Usage: ssp_server <parameters_file>" << std::endl;
    return 1;
  }

  filename = std::string(argv[1]);
#endif
    
  return ssp_server(filename.c_str());
}
#endif
