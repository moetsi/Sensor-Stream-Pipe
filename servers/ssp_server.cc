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
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
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

#ifdef SSP_WITH_DEPTHAI_SUPPORT
#include "../readers/oakd_xlink_full_reader.h"
#include "depthai/depthai.hpp"
#endif

#ifdef SSP_WITH_NVPIPE_SUPPORT
#include "../encoders/nv_encoder.h"
#endif

#ifdef SSP_WITH_KINECT_SUPPORT
#include "../readers/kinect_reader.h"
#include "../utils/kinect_utils.h"
#endif

using namespace moetsi::ssp;

//This is a global variable that will store a vector of string that describes what kind of frame types to pull for a single pull_vector_of_frames call
std::vector<std::string> frame_types_to_pull;

// Global variable to store original terminal settings (needed for hitting c)
struct termios oldt;
void setupNonBlockingInput() {
    tcgetattr(STDIN_FILENO, &oldt); //get the current terminal I/O structure
    struct termios newt = oldt;
    newt.c_lflag &= ~(ICANON); //disable canonical mode
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); //apply the new settings
    fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK); //set non-block
}

void restoreInputSettings() {
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); //restore old settings
}

volatile sig_atomic_t stop_flag = 0;

void handle_signal(int signal) {
    if (signal == SIGINT) {
        if (stop_flag) {
            restoreInputSettings(); // Restore the saved terminal settings
            std::cerr << "Forced exit" << std::endl;
            exit(signal);
        } else {
            stop_flag = 1;
            std::cerr << "Exiting after current frame..." << std::endl;
        }
    }
}

// now we make a function called initiaize that we call in ssp_server
std::tuple<std::unique_ptr<zmq::socket_t>, std::unique_ptr<IReader>, std::unordered_map<unsigned int, std::shared_ptr<IEncoder>>>
initialize(const char* filename, const char* client_key, const char* environment_name, const char* sensor_name) {

    // Initialize the parameters of the codec
    std::string codec_parameters_file = std::string(filename);
    YAML::Node codec_parameters = YAML::LoadFile(codec_parameters_file);
    std::string host = codec_parameters["general"]["host"].as<std::string>();
    unsigned int port = codec_parameters["general"]["port"].as<unsigned int>();

    // Initialize the context and socket to communicate with the client
    static auto context = std::make_unique<zmq::context_t>(1);
    auto socket = std::make_unique<zmq::socket_t>(*context, ZMQ_PUSH);

    // Do not accumulate packets if no client is connected
    socket->set(zmq::sockopt::immediate, true);

    // Do not keep packets if there is network congestion
    // socket->set(zmq::sockopt::conflate, true);
    socket->connect("tcp://" + host + ":" + std::to_string(port));

    YAML::Node general_parameters = codec_parameters["general"];
    SetupLogging(general_parameters);

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
      reader = std::unique_ptr<DummyBodyReader>(new DummyBodyReader(general_parameters["frame_source"]["parameters"]));
    }
#ifdef SSP_WITH_DEPTHAI_SUPPORT
    else if (reader_type == "oakd_xlink_full") {
        reader = std::unique_ptr<OakdXlinkFullReader>(new OakdXlinkFullReader(general_parameters["frame_source"]["parameters"], client_key, environment_name, sensor_name));
    }
#endif
     else if (reader_type == "video") {
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
      throw std::runtime_error("SSP compiled without Kinect support");
#endif
    } else if (reader_type == "iphone") {
#if TARGET_OS_IOS
      reader = std::unique_ptr<iPhoneReader>(new iPhoneReader());
#else
      throw std::runtime_error("SSP compiled without iPhone support");
#endif
    } else {
      spdlog::error("Unknown reader type: \"{}\". Supported types are "
                    "\"frames\", \"video\" and \"kinect\"",
                    reader_type);
      throw std::runtime_error("Unknown reader type");
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
        throw std::runtime_error("SSP compiled without nvenc support");
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
        throw std::runtime_error("Unknown encoder type");
      }

      encoders[unsigned(type)] = fe;
    }

    // Return the socket, reader and encoders so they can be used by other functions
    return std::make_tuple(std::move(socket), std::move(reader), std::move(encoders));
}

// This is a function that when called sets the frame_types_to_pull variable to <"rgb", "depth">
void set_frame_types_to_pull_to_rgb_depth() {
  frame_types_to_pull = {"rgb", "depth"};
  std::cout << "Frame types to pull set to: " << frame_types_to_pull[0] << ", " << frame_types_to_pull[1] << std::endl;
}

std::vector<FrameStruct> pull_vector_of_frames(std::unique_ptr<IReader>& reader, 
                                               std::unordered_map<unsigned int, std::shared_ptr<IEncoder>>& encoders)
{
  std::vector<FrameStruct> v;
  std::vector<std::shared_ptr<FrameStruct>> vO;

  while (v.empty()) {
    std::vector<std::shared_ptr<FrameStruct>> frameStruct =
        reader->GetCurrentFrame();
    for (std::shared_ptr<FrameStruct> frameStruct : frameStruct) {

      std::shared_ptr<IEncoder> frameEncoder =
          encoders[unsigned(frameStruct->frame_type)];
      if (!!frameEncoder) {
        frameEncoder->AddFrameStruct(frameStruct);
        if (frameEncoder->HasNextPacket()) {
          std::shared_ptr<FrameStruct> f =
              frameEncoder->CurrentFrameEncoded();
          vO.push_back(f);
          v.push_back(*f);
          frameEncoder->NextPacket();
        }
      }
    }
    if (reader->HasNextFrame()) {
      if (!frame_types_to_pull.empty()) {
        reader->NextFrame(frame_types_to_pull);
        frame_types_to_pull.clear();
        std::cout << "frame_types_to_pull cleared" << std::endl;
      } else {
        reader->NextFrame();
      }
    } else {
      reader->Reset();
    }
  }

  for (unsigned int i = 0; i < vO.size(); i++) {
    vO.at(i)->frame.clear();
    vO.at(i) = nullptr;
  }

  return v;
}

// New function to send vector of frames
void send_vector_of_frames(zmq::socket_t& socket, const std::vector<FrameStruct>& v)
{
  std::string message = CerealStructToString(v);

  zmq::message_t request(message.size());
  memcpy(request.data(), message.c_str(), message.size()); 
  socket.send(request, zmq::send_flags::none);
}

extern "C" void start_auto(zmq::socket_t* socket, std::unique_ptr<IReader>& reader, std::unordered_map<unsigned int, std::shared_ptr<IEncoder>>& encoders)
{

  struct termios oldt;
  tcgetattr(STDIN_FILENO, &oldt); // get current terminal settings for restoration
  setupNonBlockingInput();

  uint64_t last_time = CurrentTimeNs();
  uint64_t start_time = last_time;
  uint64_t start_frame_time = last_time;
  uint64_t sent_frames = 0;
  uint64_t processing_time = 0;

  double sent_kbytes = 0;
  double sent_latency = 0;

  unsigned int fps = reader->GetFps();    
  unsigned int frame_time = 1000000000ULL/fps;

  int c = 0;
  try {
    while (1) {

      // This is to trigger set_frame_types_to_pull_to_rgb_depth if the user presses 'c'
      char ch;
      if (read(STDIN_FILENO, &ch, 1) > 0 && ch == 'c') {
          set_frame_types_to_pull_to_rgb_depth();
      }

      //The remaining code is for pulling and sending frames
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

      std::vector<FrameStruct> v = pull_vector_of_frames(reader, encoders);

      if (!v.empty()) {
        send_vector_of_frames(*socket, v);

        sent_frames += 1;

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

        for (unsigned int i = 0; i < v.size(); i++) {
          FrameStruct f = v.at(i);
          f.frame.clear();
          spdlog::debug("\t{};{};{} sent", f.device_id, f.sensor_id,
                        f.frame_id);
        }
      }

      if (stop_flag) {
        break;
      }
    }
  } catch (...) {
        restoreInputSettings(); // ensure the settings are restored even if an error occurs
        throw; // rethrow the exception to handle it outside
    }
    restoreInputSettings(); // restore terminal settings before exiting function
}

// Updated function signature to accept 4 arguments
extern "C" SSP_EXPORT int ssp_server(const char* filename, const char* client_key, const char* environment_name, const char* sensor_name)
{
  av_log_set_level(AV_LOG_QUIET);

  try {
    std::cerr << "Initializing socket, reader and encoders" << std::endl;
    auto [socket, reader, encoders] = initialize(filename, client_key, environment_name, sensor_name);
    std::cerr << "Socket, reader and encoders initialized" << std::endl;

    start_auto(socket.get(), reader, encoders);
  } catch (YAML::Exception &e) {
    spdlog::error("Error on the YAML configuration file");
    spdlog::error(e.what());
  } catch (std::exception &e) {
    spdlog::error("General Error");
    spdlog::error(e.what());
  }

  restoreInputSettings(); // Restore terminal settings before exiting ssp_server
  return 0;
}

#ifndef SSP_PLUGIN
int main(int argc, char *argv[]) {
  signal(SIGINT, handle_signal); // Handle Ctrl-C and other relevant signals
  setupNonBlockingInput(); // Setup non-blocking input for keyboard handling

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
  // Check number of arguments
  if (argc == 2) {
    filename = std::string(argv[1]);
    int result = ssp_server(filename.c_str(), nullptr, nullptr, nullptr);
    restoreInputSettings(); // Restore terminal settings after ssp_server returns
    return result;
  } else if (argc == 5) {
    filename = std::string(argv[1]);
    const char* client_key = argv[2];
    const char* environment_name = argv[3];
    const char* sensor_name = argv[4];
    int result = ssp_server(filename.c_str(), client_key, environment_name, sensor_name);
    restoreInputSettings(); // Restore terminal settings after ssp_server returns
    return result;
  } else {
    std::cerr << "Usage: ssp_server <parameters_file> [client_key environment_name sensor_name]" << std::endl;
    return 1;
  }
#endif
  restoreInputSettings(); // Restore terminal settings on normal exit
}
#endif