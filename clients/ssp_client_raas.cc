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
// #include <backward.hpp> // Add this include for backward-cpp
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
    NetworkReader reader(9002);
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
    return 15;
}

  // This function updates the device information in the staging area and returns the number of updates.
  DLL_EXPORT int raasclUpdatesForDevice(int64_t device_id)
  {
    // Lock the mutex to ensure thread-safe access to the device_message_dictionary.
    std::unique_lock<std::mutex> lock(device_message_dictionary_mutex);
    // Attempt to find the device by its ID in the device_message_dictionary.
    auto it = device_message_dictionary.find(device_id);
    // If the device is not found, unlock the mutex and return 0 to indicate no updates.
    if (it == device_message_dictionary.end())
    {
      lock.unlock();
      return 0;
    }

    // If the device is found, proceed to move its data to the staging area.
    // Lock the mutex for the staging_dictionary to ensure thread-safe access.
    std::unique_lock<std::mutex> staging_lock(staging_dictionary_mutex);
    // Copy the device's data from the device_message_dictionary to the staging_dictionary.
    staging_dictionary[device_id] = it->second;
    // Unlock the staging_dictionary mutex as the data has been successfully copied.
    staging_lock.unlock();

    // Now that the data is safely copied to the staging area, remove the original entry from the device_message_dictionary.
    device_message_dictionary.erase(it);
    // Unlock the device_message_dictionary mutex as we're done modifying it.
    lock.unlock();

    // Return the number of detections for this device, which is stored in the second element of the pair.
    return it->second.second;
  }

  // This function retrieves the last update for a specific device and body number and stores it in the provided detection structure.
  // TODO: This could be updated to return all the updates at once in 1 struct but have not made that update
  DLL_EXPORT void raasclGetLastUpdateOut(int64_t device_id, int bodyNumber, detection_struct_t& detection)
  {
    // Lock the mutex for the staging_dictionary to ensure thread-safe access.
    std::unique_lock<std::mutex> staging_lock(staging_dictionary_mutex);
    // Attempt to find the device by its ID in the staging_dictionary.
    auto it = staging_dictionary.find(device_id);
    // Check if the device is not found or if the requested bodyNumber is out of range.
    if (it == staging_dictionary.end() || bodyNumber >= it->second.second)
    {
      // If either condition is true, unlock the mutex and exit the function as there's no relevant data to return.
      staging_lock.unlock();
      return; // No data available for this device_id or bodyNumber
    }

    // If the device and bodyNumber are valid, retrieve the detection data and store it in the provided detection structure.
    detection = it->second.first[bodyNumber];
    // Unlock the staging_dictionary mutex as we're done accessing it.
    staging_lock.unlock();
  }
}