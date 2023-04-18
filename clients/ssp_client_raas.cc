#include <atomic>
#include <chrono>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "../utils/logger.h"
#include "../readers/network_reader.h"
#include "../utils/video_utils.h"
#include "../utils/image_converter.h"
#include <backward.hpp> // Add this include for backward-cpp


#ifdef _WIN32
#include <io.h>
#else
#include <unistd.h>
#endif 

#include <zmq.hpp>
#include <fstream>

using namespace moetsi::ssp;

struct SensorData {
  double x_position;
  double y_position;
  double z_position;
  int face_id;
  int silhouette_id;
  int this_sensor_body_tracking_id;
};

struct DeviceMessage {
  int device_id;
  double timestamp;
  SensorData* sensor_data;
  int sensor_data_count;
};

std::ofstream error_log_file;

// State variables to store message data and handle threading
std::unordered_map<int, std::tuple<std::chrono::system_clock::time_point, SensorData*, int>> device_message_dictionary;
std::mutex device_message_dictionary_mutex;

void log_exception(const std::exception &e, const std::string &location) {
    std::ostringstream log_message;

    // Log the date and time
    auto now = std::chrono::system_clock::now();
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
    log_message << "[" << std::put_time(std::localtime(&now_time_t), "%Y-%m-%d %H:%M:%S") << "] ";

    // Log the error type and message
    log_message << "Exception thrown at " << location << ": ";
    log_message << typeid(e).name() << " - " << e.what() << "\n";

    // Log the stack trace
    backward::StackTrace st;
    st.load_here(32);
    backward::Printer p;
    p.snippet = true;
    p.object = true;
    p.address = true;
    p.print(st, log_message);

    // Log the message to spdlog and the error_log_file
    spdlog::error(log_message.str());
    if (error_log_file.is_open()) {
        error_log_file << log_message.str() << std::endl;
    }
}

void start_ssp_client_raas() {
  try {


    // We connect to port 9002 on local host which is the RaaS consumer port
    NetworkReader reader(9002);
    reader.init("127.0.0.1");

    std::unordered_map<std::string, std::shared_ptr<IDecoder>> decoders;

    int32_t bodyCount;
    coco_human_t bodyStruct;

    while (reader.HasNextFrame()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      coco_human_t bodyStruct;

      reader.NextFrame();
      std::vector<FrameStruct> f_list = reader.GetCurrentFrame();
      for (FrameStruct f : f_list) {

        std::string decoder_id = f.stream_id + std::to_string(f.sensor_id);

        if (f.frame_type == FrameType::FrameTypeHumanPose)
        {
          if (f.frame.size() < 4 + sizeof(coco_human_t)) {

            std::cerr << "0 size frame -> skip " << std::endl << std::flush;
            continue;

          } else {

            // f.device_id is the device_id of the device that sent the message which is an int
            int device_id = f.device_id;
            auto now = std::chrono::system_clock::now();

            // This is dummy data to fill the SensorData struct
            SensorData dummy_sensor_data{0, 0, 0, 0, 0, 0};
            std::vector<SensorData> dummy_sensor_data_vector{dummy_sensor_data};

            std::unique_lock<std::mutex> lock(device_message_dictionary_mutex);
            auto& device_message_entry = device_message_dictionary[device_id];
            std::get<0>(device_message_entry) = now;

            // Allocate memory for the sensor_data array
            SensorData* sensor_data_array = new SensorData[dummy_sensor_data_vector.size()];
            std::copy(dummy_sensor_data_vector.begin(), dummy_sensor_data_vector.end(), sensor_data_array);
            std::get<1>(device_message_entry) = sensor_data_array;
            std::get<2>(device_message_entry) = dummy_sensor_data_vector.size();  // Store the sensor_data_count
            lock.unlock();
          }
        }

      }
    }

  } catch (std::exception &e) {
    log_exception(e, "start_ssp_client_raas");
  }

}

extern "C" {
  #ifdef _WIN32
  #define DLL_EXPORT __declspec(dllexport)
  #else
  #define DLL_EXPORT __attribute__((visibility("default")))
  #endif

  DLL_EXPORT int return_four() {
    return 6;
  }

  DLL_EXPORT void open_error_log_file(const char* file_path) {
    error_log_file.open(file_path, std::ios_base::out | std::ios_base::app);
  }

  DLL_EXPORT void close_error_log_file() {
    if (error_log_file.is_open()) {
      error_log_file.close();
    }
  }

  DLL_EXPORT void start_ssp_client_raas_c_wrapper() {
    start_ssp_client_raas();
  }

  DLL_EXPORT int return_new_messages_count() {
    std::lock_guard<std::mutex> lock(device_message_dictionary_mutex);
    return device_message_dictionary.size();
  }

  DLL_EXPORT void return_new_messages(DeviceMessage* messages) {
      std::lock_guard<std::mutex> lock(device_message_dictionary_mutex);
      int index = 0;

      for (const auto& device_message_pair : device_message_dictionary) {
          int device_id = device_message_pair.first;
          auto time_and_data = device_message_pair.second;

          messages[index].device_id = device_id;
          messages[index].timestamp = std::chrono::duration<double>(std::get<0>(time_and_data).time_since_epoch()).count();

          const auto& sensor_data_array = std::get<1>(time_and_data);
          int sensor_data_count = std::get<2>(time_and_data);
          messages[index].sensor_data_count = sensor_data_count;
          messages[index].sensor_data = new SensorData[sensor_data_count];
          memcpy(messages[index].sensor_data, sensor_data_array, sizeof(SensorData) * sensor_data_count);

          // Delete the allocated memory for sensor_data_array
          delete[] sensor_data_array;

          index++;
      }

      device_message_dictionary.clear();
  }

}