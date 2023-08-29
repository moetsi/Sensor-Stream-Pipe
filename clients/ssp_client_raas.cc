#include <atomic>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "../utils/logger.h"
#include "../readers/network_reader.h"
#include "../utils/video_utils.h"
#include "../utils/image_converter.h"
#include <backward.hpp> // Add this include for backward-cpp
#include "../structs/body_struct.h"
#include "../structs/detection_struct.h"

#ifdef _WIN32
#include <io.h>
#else
#include <unistd.h>
#endif

#include <zmq.hpp>
#include <fstream>

using namespace moetsi::ssp;

// Atomic flag to control the Runner thread
std::atomic_bool kill_thread(false);

// Dictionary to store the latest message data for each device
std::unordered_map<int, std::pair<std::shared_ptr<detection_struct_t[]>, int>> device_message_dictionary;
std::mutex device_message_dictionary_mutex;

// Staging area to temporarily hold data once queried by Unity
std::unordered_map<int, std::pair<std::shared_ptr<detection_struct_t[]>, int>> staging_dictionary;
std::mutex staging_dictionary_mutex;

void start_ssp_client_raas(const std::string& ip_address) {
  try {
    // Connect to port 9002 on local host which is the RaaS consumer port
    NetworkReader reader(10200);
    reader.init(ip_address);  // Initialize the reader with the provided IP address

    while (!kill_thread && reader.HasNextFrame()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));

      reader.NextFrame();
      std::vector<FrameStruct> f_list = reader.GetCurrentFrame();
      for (FrameStruct f : f_list) {
        if (f.frame_type == FrameType::FrameTypeDetection)
        {
          int device_id = f.device_id;
          int64_t timestamp = f.frame_device_timestamp;

          int32_t bodyCount;
          memcpy(&bodyCount, &f.frame[0], sizeof(int32_t));

          // Create a shared_ptr for the detection_struct_t array
          auto detection_array = std::shared_ptr<detection_struct_t[]>(
              new detection_struct_t[bodyCount], 
              std::default_delete<detection_struct_t[]>()
          );
          for (int i = 0; i < bodyCount; i++)
          {
            detection_struct_t detection;
            memcpy(&detection, &f.frame[4 + i * sizeof(detection_struct_t)], sizeof(detection_struct_t));
            detection_array.get()[i] = detection;
          }

          std::unique_lock<std::mutex> lock(device_message_dictionary_mutex);
          device_message_dictionary[device_id] = std::make_pair(detection_array, bodyCount);
          lock.unlock();
        }
      }
    }
  } catch (std::exception &e) {
    std::cerr << "Exception: " << e.what() << std::endl;
  }
}

// Runner struct to manage the thread and its associated resources
struct Runner {
  std::thread t;

  Runner(std::function<void()> func) {
    kill_thread = false;
    t = std::thread(func);
  }

  ~Runner() {
    kill_thread = true;
    if (t.joinable()) {
      t.join();
    }
  }
};

extern "C" {
  #ifdef _WIN32
  #define DLL_EXPORT __declspec(dllexport)
  #else
  #define DLL_EXPORT __attribute__((visibility("default")))
  #endif

  DLL_EXPORT void stop_ssp_client_raas_c_wrapper() {
    kill_thread = true;
  }

  // Add a mutex for Runner instance creation and destruction
  std::mutex runner_mutex;

  DLL_EXPORT void start_ssp_client_raas_c_wrapper() {
      std::lock_guard<std::mutex> lock(runner_mutex); // Lock the mutex
      static std::shared_ptr<Runner> runner;

      if (runner) {
        runner.reset(); // Release the previous instance of Runner
      }

      runner = std::make_shared<Runner>([]() {
        start_ssp_client_raas("127.0.0.1"); // Default IP
      });
  }

  DLL_EXPORT void start_ssp_client_raas_c_wrapper_with_ip(const char* ip_address_cstr) {
      std::lock_guard<std::mutex> lock(runner_mutex); // Lock the mutex
      static std::shared_ptr<Runner> runner;

      if (runner) {
        runner.reset(); // Release the previous instance of Runner
      }

      // Convert C string to std::string and pass it to the Runner's lambda function
      std::string ip_address(ip_address_cstr);
      runner = std::make_shared<Runner>([ip_address]() {
        start_ssp_client_raas(ip_address);
      });
  }

  DLL_EXPORT int return_four() {
    return 14;
}

  DLL_EXPORT int raasclUpdatesForDevice(int64_t device_id)
  {
    std::unique_lock<std::mutex> lock(device_message_dictionary_mutex);
    auto it = device_message_dictionary.find(device_id);
    if (it == device_message_dictionary.end())
    {
      lock.unlock();
      return 0;
    }

    // Move the data to the staging area
    std::unique_lock<std::mutex> staging_lock(staging_dictionary_mutex);
    staging_dictionary[device_id] = it->second;
    staging_lock.unlock();

    // Remove the entry from the original dictionary
    device_message_dictionary.erase(it);
    lock.unlock();

    return it->second.second; // Return the number of detections
  }

  DLL_EXPORT void raasclGetLastUpdateOut(int64_t device_id, int bodyNumber, detection_struct_t& detection)
  {
    std::unique_lock<std::mutex> staging_lock(staging_dictionary_mutex);
    auto it = staging_dictionary.find(device_id);
    if (it == staging_dictionary.end() || bodyNumber >= it->second.second)
    {
      staging_lock.unlock();
      return; // No data available for this device_id or bodyNumber
    }

    detection = it->second.first[bodyNumber];
    staging_lock.unlock();
  }
}