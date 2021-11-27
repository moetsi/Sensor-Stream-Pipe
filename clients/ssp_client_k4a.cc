/**
 * \file ssp_client_k4a.cc @brief SSP client with lib k4a
 */
// Created by amourao on 26-06-2019.

#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <io.h>
#define SSP_EXPORT __declspec(dllexport)
#else
#include <unistd.h>
#define SSP_EXPORT
#endif

#include <k4a/k4a.h>
#include <opencv2/imgproc.hpp>
#include <zmq.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

#include "../utils/logger.h"

#include "../readers/network_reader.h"
#include "../utils/kinect_utils.h"

using namespace moetsi::ssp;

typedef struct _custom_k4abt_body_t
{
  int32_t Id;
  float pelvis_x;
  float pelvis_y;
  float pelvis_z;
  float pelvis_QX;
  float pelvis_QY;
  float pelvis_QZ;
  float pelvis_QW;
  uint8_t pelvis_conf;
  float spine_navel_x;
  float spine_navel_y;
  float spine_navel_z;
  float spine_navel_QX;
  float spine_navel_QY;
  float spine_navel_QZ;
  float spine_navel_QW;
  uint8_t spine_navel_conf;
  float spine_chest_x;
  float spine_chest_y;
  float spine_chest_z;
  float spine_chest_QX;
  float spine_chest_QY;
  float spine_chest_QZ;
  float spine_chest_QW;
  uint8_t spine_chest_conf;
  float neck_x;
  float neck_y;
  float neck_z;
  float neck_QX;
  float neck_QY;
  float neck_QZ;
  float neck_QW;
  uint8_t neck_conf;
  float clavicle_left_x;
  float clavicle_left_y;
  float clavicle_left_z;
  float clavicle_left_QX;
  float clavicle_left_QY;
  float clavicle_left_QZ;
  float clavicle_left_QW;
  uint8_t clavicle_left_conf;
  float shoulder_left_x;
  float shoulder_left_y;
  float shoulder_left_z;
  float shoulder_left_QX;
  float shoulder_left_QY;
  float shoulder_left_QZ;
  float shoulder_left_QW;
  uint8_t shoulder_left_conf;
  float elbow_left_x;
  float elbow_left_y;
  float elbow_left_z;
  float elbow_left_QX;
  float elbow_left_QY;
  float elbow_left_QZ;
  float elbow_left_QW;
  uint8_t elbow_left_conf;
  float wrist_left_x;
  float wrist_left_y;
  float wrist_left_z;
  float wrist_left_QX;
  float wrist_left_QY;
  float wrist_left_QZ;
  float wrist_left_QW;
  uint8_t wrist_left_conf;
  float hand_left_x;
  float hand_left_y;
  float hand_left_z;
  float hand_left_QX;
  float hand_left_QY;
  float hand_left_QZ;
  float hand_left_QW;
  uint8_t hand_left_conf;
  float handtip_left_x;
  float handtip_left_y;
  float handtip_left_z;
  float handtip_left_QX;
  float handtip_left_QY;
  float handtip_left_QZ;
  float handtip_left_QW;
  uint8_t handtip_left_conf;
  float thumb_left_x;
  float thumb_left_y;
  float thumb_left_z;
  float thumb_left_QX;
  float thumb_left_QY;
  float thumb_left_QZ;
  float thumb_left_QW;
  uint8_t thumb_left_conf;
  float clavicle_right_x;
  float clavicle_right_y;
  float clavicle_right_z;
  float clavicle_right_QX;
  float clavicle_right_QY;
  float clavicle_right_QZ;
  float clavicle_right_QW;
  uint8_t clavicle_right_conf;
  float shoulder_right_x;
  float shoulder_right_y;
  float shoulder_right_z;
  float shoulder_right_QX;
  float shoulder_right_QY;
  float shoulder_right_QZ;
  float shoulder_right_QW;
  uint8_t shoulder_right_conf;
  float elbow_right_x;
  float elbow_right_y;
  float elbow_right_z;
  float elbow_right_QX;
  float elbow_right_QY;
  float elbow_right_QZ;
  float elbow_right_QW;
  uint8_t elbow_right_conf;
  float wrist_right_x;
  float wrist_right_y;
  float wrist_right_z;
  float wrist_right_QX;
  float wrist_right_QY;
  float wrist_right_QZ;
  float wrist_right_QW;
  uint8_t wrist_right_conf;
  float hand_right_x;
  float hand_right_y;
  float hand_right_z;
  float hand_right_QX;
  float hand_right_QY;
  float hand_right_QZ;
  float hand_right_QW;
  uint8_t hand_right_conf;
  float handtip_right_x;
  float handtip_right_y;
  float handtip_right_z;
  float handtip_right_QX;
  float handtip_right_QY;
  float handtip_right_QZ;
  float handtip_right_QW;
  uint8_t handtip_right_conf;
  float thumb_right_x;
  float thumb_right_y;
  float thumb_right_z;
  float thumb_right_QX;
  float thumb_right_QY;
  float thumb_right_QZ;
  float thumb_right_QW;
  uint8_t thumb_right_conf;
  float hip_left_x;
  float hip_left_y;
  float hip_left_z;
  float hip_left_QX;
  float hip_left_QY;
  float hip_left_QZ;
  float hip_left_QW;
  uint8_t hip_left_conf;
  float knee_left_x;
  float knee_left_y;
  float knee_left_z;
  float knee_left_QX;
  float knee_left_QY;
  float knee_left_QZ;
  float knee_left_QW;
  uint8_t knee_left_conf;
  float ankle_left_x;
  float ankle_left_y;
  float ankle_left_z;
  float ankle_left_QX;
  float ankle_left_QY;
  float ankle_left_QZ;
  float ankle_left_QW;
  uint8_t ankle_left_conf;
  float foot_left_x;
  float foot_left_y;
  float foot_left_z;
  float foot_left_QX;
  float foot_left_QY;
  float foot_left_QZ;
  float foot_left_QW;
  uint8_t foot_left_conf;
  float hip_right_x;
  float hip_right_y;
  float hip_right_z;
  float hip_right_QX;
  float hip_right_QY;
  float hip_right_QZ;
  float hip_right_QW;
  uint8_t hip_right_conf;
  float knee_right_x;
  float knee_right_y;
  float knee_right_z;
  float knee_right_QX;
  float knee_right_QY;
  float knee_right_QZ;
  float knee_right_QW;
  uint8_t knee_right_conf;
  float ankle_right_x;
  float ankle_right_y;
  float ankle_right_z;
  float ankle_right_QX;
  float ankle_right_QY;
  float ankle_right_QZ;
  float ankle_right_QW;
  uint8_t ankle_right_conf;
  float foot_right_x;
  float foot_right_y;
  float foot_right_z;
  float foot_right_QX;
  float foot_right_QY;
  float foot_right_QZ;
  float foot_right_QW;
  uint8_t foot_right_conf;
  float head_x;
  float head_y;
  float head_z;
  float head_QX;
  float head_QY;
  float head_QZ;
  float head_QW;
  uint8_t head_conf;
  float nose_x;
  float nose_y;
  float nose_z;
  float nose_QX;
  float nose_QY;
  float nose_QZ;

  float nose_QW;
  uint8_t nose_conf;
  float eye_left_x;
  float eye_left_y;
  float eye_left_z;
  float eye_left_QX;
  float eye_left_QY;
  float eye_left_QZ;
  float eye_left_QW;
  uint8_t eye_left_conf;
  float ear_left_x;
  float ear_left_y;
  float ear_left_z;
  float ear_left_QX;
  float ear_left_QY;
  float ear_left_QZ;
  float ear_left_QW;
  uint8_t ear_left_conf;
  float eye_right_x;
  float eye_right_y;
  float eye_right_z;
  float eye_right_QX;
  float eye_right_QY;
  float eye_right_QZ;
  float eye_right_QW;
  uint8_t eye_right_conf;
  float ear_right_x;
  float ear_right_y;
  float ear_right_z;
  float ear_right_QX;
  float ear_right_QY;
  float ear_right_QZ;
  float ear_right_QW;
  uint8_t ear_right_conf;
} custom_k4abt_body_t;

class BodyTracker
{
public:
    BodyTracker(int port);
    ~BodyTracker();

    int update();
    int getBodyCount() const;
    int getBodiesStruct(k4abt_body_t* pBodies, int n) const;
    custom_k4abt_body_t getCustomBodiesStruct(int n) const;
    int getBodies(k4abt_skeleton_t* pSkeletons, int* pIds, int n) const;

private:
    NetworkReader* m_reader;
    k4a::calibration m_sensor_calibration;
    bool m_calibration_set;
    k4abt::tracker m_tracker;
    std::unordered_map<std::string, std::shared_ptr<IDecoder>> m_decoders;
    std::vector< k4abt_body_t> m_bodies;
    mutable std::mutex m_mutex;
};

BodyTracker::BodyTracker(int port)
{
    spdlog::set_level(spdlog::level::off);
    av_log_set_level(AV_LOG_QUIET);

    m_calibration_set = false;

    m_reader = new NetworkReader(port);
    m_reader->init();
}

BodyTracker::~BodyTracker()
{
    delete m_reader;
}

int BodyTracker::update()
{
  try {
    if (m_reader->HasNextFrame()) {
      m_reader->NextFrame();
      std::vector<FrameStruct> f_list = m_reader->GetCurrentFrame();
      for (FrameStruct f : f_list) {
        std::string decoder_id = f.stream_id + std::to_string(uint64_t(f.sensor_id));

        if (f.camera_calibration_data.type == CameraCalibrationType::CameraCalibrationTypeKinect && m_calibration_set == false) {
          const k4a_depth_mode_t d = static_cast<const k4a_depth_mode_t>(f.camera_calibration_data.extra_data[0]);
          const k4a_color_resolution_t r =
            static_cast<const k4a_color_resolution_t>(f.camera_calibration_data.extra_data[1]);

          m_sensor_calibration = k4a::calibration::get_from_raw(
            reinterpret_cast<char *>(&f.camera_calibration_data.data[0]),
            f.camera_calibration_data.data.size(), d, r);

          m_calibration_set = true;
          m_tracker = k4abt::tracker::create(m_sensor_calibration);
        }
      }

      k4a::capture sensor_capture = k4a::capture::create();
      FrameStructToK4A(f_list, sensor_capture, m_decoders);

      if (!m_tracker.enqueue_capture(sensor_capture)) {
        // It should never hit timeout when K4A_WAIT_INFINITE is set.
        spdlog::error("Error adding capture to tracker process queue timeout!");
        return -1;
      }

      k4abt::frame body_frame = m_tracker.pop_result();
      if (body_frame != nullptr) {
        m_mutex.lock();
        size_t num_bodies = body_frame.get_num_bodies();
        m_bodies.resize(num_bodies);

        // Store bodies
        for (size_t i = 0; i < num_bodies; i++)
          m_bodies[i] = body_frame.get_body(i);
        m_mutex.unlock();
      }
      else {
        spdlog::error("Pop body frame result time out!!");
        return -1;
      }
    }
  }
  catch (std::exception &e) {
    spdlog::error(e.what());
    return -1;
  }

  return 0;
}

int BodyTracker::getBodyCount() const
{
  m_mutex.lock();
  int num = m_bodies.size();
  m_mutex.unlock();

  return num;
}

int BodyTracker::getBodiesStruct(k4abt_body_t* pBodies, int n) const
{
  // If the number has changed since call to getBodyCount
  // Fill at most n bodies

  m_mutex.lock();
  int num = std::min(n, (int)m_bodies.size());
  memcpy(pBodies, &m_bodies[0], num * sizeof(k4abt_body_t));
  m_mutex.unlock();

  return num;
}

custom_k4abt_body_t BodyTracker::getCustomBodiesStruct(int n) const
{
  m_mutex.lock();
  custom_k4abt_body_t customBody;
  // As custom_k4abt_body_t and k4abt_body_t have the same packing/alignment
  // we copy it
  if (n >= 0 && n < m_bodies.size())
    customBody = *(custom_k4abt_body_t*)&m_bodies[n];
  else
    memset(&customBody, 0, sizeof(customBody));
  m_mutex.unlock();
  return customBody;
}

int BodyTracker::getBodies(k4abt_skeleton_t* pSkeletons, int* pIds, int n) const
{
  // If the number has changed since call to getBodyCount
  // Fill at most n bodies

  m_mutex.lock();
  int num = std::min(n, (int)m_bodies.size());
  for (int i = 0; i < num; i++)
  {
    pIds[i] = m_bodies[i].id;
    pSkeletons[i] = m_bodies[i].skeleton;
  }
  m_mutex.unlock();

  return num;
}

BodyTracker *gTracker = NULL;
std::thread gUpdateThread;
bool gStop = false;

extern "C" SSP_EXPORT int open_k4a(int port)
{
    if (gTracker != NULL)
        return -1;

    gTracker = new BodyTracker(port);

    return 0;
}

extern "C" SSP_EXPORT int close_k4a()
{
    if (gTracker == NULL)
        return -1;

    delete gTracker;
    gTracker = NULL;
    return 0;
}

void update()
{
  while (!gStop && gTracker != NULL)
  {
    gTracker->update();
  }
}

extern "C" SSP_EXPORT int start_k4a(int port)
{
  if (gTracker != NULL)
    return -1;

  gTracker = new BodyTracker(port);

  // Start update thread
  gStop = false;
  gUpdateThread = std::thread(update);

  return 0;
}

extern "C" SSP_EXPORT int stop_k4a()
{
  if (gTracker == NULL)
    return -1;

  gStop = true;
  if (gUpdateThread.joinable())
    gUpdateThread.join();

  delete gTracker;
  gTracker = NULL;
  return 0;
}

extern "C" SSP_EXPORT int update_k4a()
{
    if (gTracker == NULL)
        return -1;

    gTracker->update();

    return 0;
}

extern "C" SSP_EXPORT int getBodyCount()
{
    return gTracker->getBodyCount();
}

extern "C" SSP_EXPORT int getBodiesStruct(k4abt_body_t* pBodies, int n)
{
  return gTracker->getBodiesStruct(pBodies, n);
}

extern "C" SSP_EXPORT custom_k4abt_body_t getCustomBodiesStruct(int n)
{
  return gTracker->getCustomBodiesStruct(n);
}

extern "C" SSP_EXPORT int getBodies(k4abt_skeleton_t* pSkeletons, int* pIds, int n)
{
    return gTracker->getBodies(pSkeletons, pIds, n);
}

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

  assert(body_index_map.get_stride_uint8_ts() ==
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

#ifndef SSP_PLUGIN
int main(int argc, char *argv[]) {

  spdlog::set_level(spdlog::level::debug);
  av_log_set_level(AV_LOG_QUIET);

  srand(time(NULL));

  try {

    if (argc < 2) {
      std::cerr << "Usage: ssp_client <port> (<log level>) (<log file>)"
                << std::endl;
      return 1;
    }
    std::string log_level = "debug";
    std::string log_file = "";

    if (argc > 2)
      log_level = argv[2];
    if (argc > 3)
      log_file = argv[3];

    int port = std::stoi(argv[1]);
    NetworkReader reader(port);

    reader.init();

    k4a::calibration sensor_calibration;
    bool calibration_set = false;
    k4abt::tracker tracker;

    std::unordered_map<std::string, std::shared_ptr<IDecoder>> decoders;

    while (reader.HasNextFrame()) {
      reader.NextFrame();
      std::vector<FrameStruct> f_list = reader.GetCurrentFrame();
      for (FrameStruct f : f_list) {
        std::string decoder_id = f.stream_id + std::to_string(f.sensor_id);

        if (f.camera_calibration_data.type == CameraCalibrationType::CameraCalibrationTypeKinect && calibration_set == false) {

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
      FrameStructToK4A(f_list, sensor_capture, decoders);

      if (!tracker.enqueue_capture(sensor_capture)) {
        // It should never hit timeout when K4A_WAIT_INFINITE is set.
        spdlog::error("Error adding capture to tracker process queue timeout!");
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
    }

  } catch (std::exception &e) {
    spdlog::error(e.what());
  }

  return 0;
}
#endif
