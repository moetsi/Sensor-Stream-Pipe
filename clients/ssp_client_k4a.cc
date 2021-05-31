//
// Created by amourao on 26-06-2019.
//

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
  BYTE pelvis_conf;
  float spine_navel_x;
  float spine_navel_y;
  float spine_navel_z;
  float spine_navel_QX;
  float spine_navel_QY;
  float spine_navel_QZ;
  float spine_navel_QW;
  BYTE spine_navel_conf;
  float spine_chest_x;
  float spine_chest_y;
  float spine_chest_z;
  float spine_chest_QX;
  float spine_chest_QY;
  float spine_chest_QZ;
  float spine_chest_QW;
  BYTE spine_chest_conf;
  float neck_x;
  float neck_y;
  float neck_z;
  float neck_QX;
  float neck_QY;
  float neck_QZ;
  float neck_QW;
  BYTE neck_conf;
  float clavicle_left_x;
  float clavicle_left_y;
  float clavicle_left_z;
  float clavicle_left_QX;
  float clavicle_left_QY;
  float clavicle_left_QZ;
  float clavicle_left_QW;
  BYTE clavicle_left_conf;
  float shoulder_left_x;
  float shoulder_left_y;
  float shoulder_left_z;
  float shoulder_left_QX;
  float shoulder_left_QY;
  float shoulder_left_QZ;
  float shoulder_left_QW;
  BYTE shoulder_left_conf;
  float elbow_left_x;
  float elbow_left_y;
  float elbow_left_z;
  float elbow_left_QX;
  float elbow_left_QY;
  float elbow_left_QZ;
  float elbow_left_QW;
  BYTE elbow_left_conf;
  float wrist_left_x;
  float wrist_left_y;
  float wrist_left_z;
  float wrist_left_QX;
  float wrist_left_QY;
  float wrist_left_QZ;
  float wrist_left_QW;
  BYTE wrist_left_conf;
  float hand_left_x;
  float hand_left_y;
  float hand_left_z;
  float hand_left_QX;
  float hand_left_QY;
  float hand_left_QZ;
  float hand_left_QW;
  BYTE hand_left_conf;
  float handtip_left_x;
  float handtip_left_y;
  float handtip_left_z;
  float handtip_left_QX;
  float handtip_left_QY;
  float handtip_left_QZ;
  float handtip_left_QW;
  BYTE handtip_left_conf;
  float thumb_left_x;
  float thumb_left_y;
  float thumb_left_z;
  float thumb_left_QX;
  float thumb_left_QY;
  float thumb_left_QZ;
  float thumb_left_QW;
  BYTE thumb_left_conf;
  float clavicle_right_x;
  float clavicle_right_y;
  float clavicle_right_z;
  float clavicle_right_QX;
  float clavicle_right_QY;
  float clavicle_right_QZ;
  float clavicle_right_QW;
  BYTE clavicle_right_conf;
  float shoulder_right_x;
  float shoulder_right_y;
  float shoulder_right_z;
  float shoulder_right_QX;
  float shoulder_right_QY;
  float shoulder_right_QZ;
  float shoulder_right_QW;
  BYTE shoulder_right_conf;
  float elbow_right_x;
  float elbow_right_y;
  float elbow_right_z;
  float elbow_right_QX;
  float elbow_right_QY;
  float elbow_right_QZ;
  float elbow_right_QW;
  BYTE elbow_right_conf;
  float wrist_right_x;
  float wrist_right_y;
  float wrist_right_z;
  float wrist_right_QX;
  float wrist_right_QY;
  float wrist_right_QZ;
  float wrist_right_QW;
  BYTE wrist_right_conf;
  float hand_right_x;
  float hand_right_y;
  float hand_right_z;
  float hand_right_QX;
  float hand_right_QY;
  float hand_right_QZ;
  float hand_right_QW;
  BYTE hand_right_conf;
  float handtip_right_x;
  float handtip_right_y;
  float handtip_right_z;
  float handtip_right_QX;
  float handtip_right_QY;
  float handtip_right_QZ;
  float handtip_right_QW;
  BYTE handtip_right_conf;
  float thumb_right_x;
  float thumb_right_y;
  float thumb_right_z;
  float thumb_right_QX;
  float thumb_right_QY;
  float thumb_right_QZ;
  float thumb_right_QW;
  BYTE thumb_right_conf;
  float hip_left_x;
  float hip_left_y;
  float hip_left_z;
  float hip_left_QX;
  float hip_left_QY;
  float hip_left_QZ;
  float hip_left_QW;
  BYTE hip_left_conf;
  float knee_left_x;
  float knee_left_y;
  float knee_left_z;
  float knee_left_QX;
  float knee_left_QY;
  float knee_left_QZ;
  float knee_left_QW;
  BYTE knee_left_conf;
  float ankle_left_x;
  float ankle_left_y;
  float ankle_left_z;
  float ankle_left_QX;
  float ankle_left_QY;
  float ankle_left_QZ;
  float ankle_left_QW;
  BYTE ankle_left_conf;
  float foot_left_x;
  float foot_left_y;
  float foot_left_z;
  float foot_left_QX;
  float foot_left_QY;
  float foot_left_QZ;
  float foot_left_QW;
  BYTE foot_left_conf;
  float hip_right_x;
  float hip_right_y;
  float hip_right_z;
  float hip_right_QX;
  float hip_right_QY;
  float hip_right_QZ;
  float hip_right_QW;
  BYTE hip_right_conf;
  float knee_right_x;
  float knee_right_y;
  float knee_right_z;
  float knee_right_QX;
  float knee_right_QY;
  float knee_right_QZ;
  float knee_right_QW;
  BYTE knee_right_conf;
  float ankle_right_x;
  float ankle_right_y;
  float ankle_right_z;
  float ankle_right_QX;
  float ankle_right_QY;
  float ankle_right_QZ;
  float ankle_right_QW;
  BYTE ankle_right_conf;
  float foot_right_x;
  float foot_right_y;
  float foot_right_z;
  float foot_right_QX;
  float foot_right_QY;
  float foot_right_QZ;
  float foot_right_QW;
  BYTE foot_right_conf;
  float head_x;
  float head_y;
  float head_z;
  float head_QX;
  float head_QY;
  float head_QZ;
  float head_QW;
  BYTE head_conf;
  float nose_x;
  float nose_y;
  float nose_z;
  float nose_QX;
  float nose_QY;
  float nose_QZ;

  float nose_QW;
  BYTE nose_conf;
  float eye_left_x;
  float eye_left_y;
  float eye_left_z;
  float eye_left_QX;
  float eye_left_QY;
  float eye_left_QZ;
  float eye_left_QW;
  BYTE eye_left_conf;
  float ear_left_x;
  float ear_left_y;
  float ear_left_z;
  float ear_left_QX;
  float ear_left_QY;
  float ear_left_QZ;
  float ear_left_QW;
  BYTE ear_left_conf;
  float eye_right_x;
  float eye_right_y;
  float eye_right_z;
  float eye_right_QX;
  float eye_right_QY;
  float eye_right_QZ;
  float eye_right_QW;
  BYTE eye_right_conf;
  float ear_right_x;
  float ear_right_y;
  float ear_right_z;
  float ear_right_QX;
  float ear_right_QY;
  float ear_right_QZ;
  float ear_right_QW;
  BYTE ear_right_conf;
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
    std::vector< custom_k4abt_body_t> m_customBodies;
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
        std::string decoder_id = f.stream_id + std::to_string(f.sensor_id);

        if (f.camera_calibration_data.type == 0 && m_calibration_set == false) {
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
        m_customBodies.resize(num_bodies);

        // Store bodies
        for (size_t i = 0; i < num_bodies; i++)
        {
          _k4abt_body_t transformingBody = body_frame.get_body(i);
          m_bodies[i] = transformingBody;

          custom_k4abt_body_t transformedCustomBody;
          transformedCustomBody.Id = transformingBody.id;
          transformedCustomBody.pelvis_x = transformingBody.skeleton.joints[0].position.xyz.x;
          transformedCustomBody.pelvis_y = transformingBody.skeleton.joints[0].position.xyz.y;
          transformedCustomBody.pelvis_z = transformingBody.skeleton.joints[0].position.xyz.z;
          transformedCustomBody.pelvis_QX = transformingBody.skeleton.joints[0].orientation.wxyz.x;
          transformedCustomBody.pelvis_QY = transformingBody.skeleton.joints[0].orientation.wxyz.y;
          transformedCustomBody.pelvis_QZ = transformingBody.skeleton.joints[0].orientation.wxyz.z;
          transformedCustomBody.pelvis_QW = transformingBody.skeleton.joints[0].orientation.wxyz.w;
          transformedCustomBody.pelvis_conf = transformingBody.skeleton.joints[0].confidence_level;
          transformedCustomBody.spine_navel_x = transformingBody.skeleton.joints[1].position.xyz.x;
          transformedCustomBody.spine_navel_y = transformingBody.skeleton.joints[1].position.xyz.y;
          transformedCustomBody.spine_navel_z = transformingBody.skeleton.joints[1].position.xyz.z;
          transformedCustomBody.spine_navel_QX = transformingBody.skeleton.joints[1].orientation.wxyz.x;
          transformedCustomBody.spine_navel_QY = transformingBody.skeleton.joints[1].orientation.wxyz.y;
          transformedCustomBody.spine_navel_QZ = transformingBody.skeleton.joints[1].orientation.wxyz.z;
          transformedCustomBody.spine_navel_QW = transformingBody.skeleton.joints[1].orientation.wxyz.w;
          transformedCustomBody.spine_navel_conf = transformingBody.skeleton.joints[1].confidence_level;
          transformedCustomBody.spine_chest_x = transformingBody.skeleton.joints[2].position.xyz.x;
          transformedCustomBody.spine_chest_y = transformingBody.skeleton.joints[2].position.xyz.y;
          transformedCustomBody.spine_chest_z = transformingBody.skeleton.joints[2].position.xyz.z;
          transformedCustomBody.spine_chest_QX = transformingBody.skeleton.joints[2].orientation.wxyz.x;
          transformedCustomBody.spine_chest_QY = transformingBody.skeleton.joints[2].orientation.wxyz.y;
          transformedCustomBody.spine_chest_QZ = transformingBody.skeleton.joints[2].orientation.wxyz.z;
          transformedCustomBody.spine_chest_QW = transformingBody.skeleton.joints[2].orientation.wxyz.w;
          transformedCustomBody.spine_chest_conf = transformingBody.skeleton.joints[2].confidence_level;
          transformedCustomBody.neck_x = transformingBody.skeleton.joints[3].position.xyz.x;
          transformedCustomBody.neck_y = transformingBody.skeleton.joints[3].position.xyz.y;
          transformedCustomBody.neck_z = transformingBody.skeleton.joints[3].position.xyz.z;
          transformedCustomBody.neck_QX = transformingBody.skeleton.joints[3].orientation.wxyz.x;
          transformedCustomBody.neck_QY = transformingBody.skeleton.joints[3].orientation.wxyz.y;
          transformedCustomBody.neck_QZ = transformingBody.skeleton.joints[3].orientation.wxyz.z;
          transformedCustomBody.neck_QW = transformingBody.skeleton.joints[3].orientation.wxyz.w;
          transformedCustomBody.neck_conf = transformingBody.skeleton.joints[3].confidence_level;
          transformedCustomBody.clavicle_left_x = transformingBody.skeleton.joints[4].position.xyz.x;
          transformedCustomBody.clavicle_left_y = transformingBody.skeleton.joints[4].position.xyz.y;
          transformedCustomBody.clavicle_left_z = transformingBody.skeleton.joints[4].position.xyz.z;
          transformedCustomBody.clavicle_left_QX = transformingBody.skeleton.joints[4].orientation.wxyz.x;
          transformedCustomBody.clavicle_left_QY = transformingBody.skeleton.joints[4].orientation.wxyz.y;
          transformedCustomBody.clavicle_left_QZ = transformingBody.skeleton.joints[4].orientation.wxyz.z;
          transformedCustomBody.clavicle_left_QW = transformingBody.skeleton.joints[4].orientation.wxyz.w;
          transformedCustomBody.clavicle_left_conf = transformingBody.skeleton.joints[4].confidence_level;
          transformedCustomBody.shoulder_left_x = transformingBody.skeleton.joints[5].position.xyz.x;
          transformedCustomBody.shoulder_left_y = transformingBody.skeleton.joints[5].position.xyz.y;
          transformedCustomBody.shoulder_left_z = transformingBody.skeleton.joints[5].position.xyz.z;
          transformedCustomBody.shoulder_left_QX = transformingBody.skeleton.joints[5].orientation.wxyz.x;
          transformedCustomBody.shoulder_left_QY = transformingBody.skeleton.joints[5].orientation.wxyz.y;
          transformedCustomBody.shoulder_left_QZ = transformingBody.skeleton.joints[5].orientation.wxyz.z;
          transformedCustomBody.shoulder_left_QW = transformingBody.skeleton.joints[5].orientation.wxyz.w;
          transformedCustomBody.shoulder_left_conf = transformingBody.skeleton.joints[5].confidence_level;
          transformedCustomBody.elbow_left_x = transformingBody.skeleton.joints[6].position.xyz.x;
          transformedCustomBody.elbow_left_y = transformingBody.skeleton.joints[6].position.xyz.y;
          transformedCustomBody.elbow_left_z = transformingBody.skeleton.joints[6].position.xyz.z;
          transformedCustomBody.elbow_left_QX = transformingBody.skeleton.joints[6].orientation.wxyz.x;
          transformedCustomBody.elbow_left_QY = transformingBody.skeleton.joints[6].orientation.wxyz.y;
          transformedCustomBody.elbow_left_QZ = transformingBody.skeleton.joints[6].orientation.wxyz.z;
          transformedCustomBody.elbow_left_QW = transformingBody.skeleton.joints[6].orientation.wxyz.w;
          transformedCustomBody.elbow_left_conf = transformingBody.skeleton.joints[6].confidence_level;
          transformedCustomBody.wrist_left_x = transformingBody.skeleton.joints[7].position.xyz.x;
          transformedCustomBody.wrist_left_y = transformingBody.skeleton.joints[7].position.xyz.y;
          transformedCustomBody.wrist_left_z = transformingBody.skeleton.joints[7].position.xyz.z;
          transformedCustomBody.wrist_left_QX = transformingBody.skeleton.joints[7].orientation.wxyz.x;
          transformedCustomBody.wrist_left_QY = transformingBody.skeleton.joints[7].orientation.wxyz.y;
          transformedCustomBody.wrist_left_QZ = transformingBody.skeleton.joints[7].orientation.wxyz.z;
          transformedCustomBody.wrist_left_QW = transformingBody.skeleton.joints[7].orientation.wxyz.w;
          transformedCustomBody.wrist_left_conf = transformingBody.skeleton.joints[7].confidence_level;
          transformedCustomBody.hand_left_x = transformingBody.skeleton.joints[8].position.xyz.x;
          transformedCustomBody.hand_left_y = transformingBody.skeleton.joints[8].position.xyz.y;
          transformedCustomBody.hand_left_z = transformingBody.skeleton.joints[8].position.xyz.z;
          transformedCustomBody.hand_left_QX = transformingBody.skeleton.joints[8].orientation.wxyz.x;
          transformedCustomBody.hand_left_QY = transformingBody.skeleton.joints[8].orientation.wxyz.y;
          transformedCustomBody.hand_left_QZ = transformingBody.skeleton.joints[8].orientation.wxyz.z;
          transformedCustomBody.hand_left_QW = transformingBody.skeleton.joints[8].orientation.wxyz.w;
          transformedCustomBody.hand_left_conf = transformingBody.skeleton.joints[8].confidence_level;
          transformedCustomBody.handtip_left_x = transformingBody.skeleton.joints[9].position.xyz.x;
          transformedCustomBody.handtip_left_y = transformingBody.skeleton.joints[9].position.xyz.y;
          transformedCustomBody.handtip_left_z = transformingBody.skeleton.joints[9].position.xyz.z;
          transformedCustomBody.handtip_left_QX = transformingBody.skeleton.joints[9].orientation.wxyz.x;
          transformedCustomBody.handtip_left_QY = transformingBody.skeleton.joints[9].orientation.wxyz.y;
          transformedCustomBody.handtip_left_QZ = transformingBody.skeleton.joints[9].orientation.wxyz.z;
          transformedCustomBody.handtip_left_QW = transformingBody.skeleton.joints[9].orientation.wxyz.w;
          transformedCustomBody.handtip_left_conf = transformingBody.skeleton.joints[9].confidence_level;
          transformedCustomBody.thumb_left_x = transformingBody.skeleton.joints[10].position.xyz.x;
          transformedCustomBody.thumb_left_y = transformingBody.skeleton.joints[10].position.xyz.y;
          transformedCustomBody.thumb_left_z = transformingBody.skeleton.joints[10].position.xyz.z;
          transformedCustomBody.thumb_left_QX = transformingBody.skeleton.joints[10].orientation.wxyz.x;
          transformedCustomBody.thumb_left_QY = transformingBody.skeleton.joints[10].orientation.wxyz.y;
          transformedCustomBody.thumb_left_QZ = transformingBody.skeleton.joints[10].orientation.wxyz.z;
          transformedCustomBody.thumb_left_QW = transformingBody.skeleton.joints[10].orientation.wxyz.w;
          transformedCustomBody.thumb_left_conf = transformingBody.skeleton.joints[10].confidence_level;
          transformedCustomBody.clavicle_right_x = transformingBody.skeleton.joints[11].position.xyz.x;
          transformedCustomBody.clavicle_right_y = transformingBody.skeleton.joints[11].position.xyz.y;
          transformedCustomBody.clavicle_right_z = transformingBody.skeleton.joints[11].position.xyz.z;
          transformedCustomBody.clavicle_right_QX = transformingBody.skeleton.joints[11].orientation.wxyz.x;
          transformedCustomBody.clavicle_right_QY = transformingBody.skeleton.joints[11].orientation.wxyz.y;
          transformedCustomBody.clavicle_right_QZ = transformingBody.skeleton.joints[11].orientation.wxyz.z;
          transformedCustomBody.clavicle_right_QW = transformingBody.skeleton.joints[11].orientation.wxyz.w;
          transformedCustomBody.clavicle_right_conf = transformingBody.skeleton.joints[11].confidence_level;
          transformedCustomBody.shoulder_right_x = transformingBody.skeleton.joints[12].position.xyz.x;
          transformedCustomBody.shoulder_right_y = transformingBody.skeleton.joints[12].position.xyz.y;
          transformedCustomBody.shoulder_right_z = transformingBody.skeleton.joints[12].position.xyz.z;
          transformedCustomBody.shoulder_right_QX = transformingBody.skeleton.joints[12].orientation.wxyz.x;
          transformedCustomBody.shoulder_right_QY = transformingBody.skeleton.joints[12].orientation.wxyz.y;
          transformedCustomBody.shoulder_right_QZ = transformingBody.skeleton.joints[12].orientation.wxyz.z;
          transformedCustomBody.shoulder_right_QW = transformingBody.skeleton.joints[12].orientation.wxyz.w;
          transformedCustomBody.shoulder_right_conf = transformingBody.skeleton.joints[12].confidence_level;
          transformedCustomBody.elbow_right_x = transformingBody.skeleton.joints[13].position.xyz.x;
          transformedCustomBody.elbow_right_y = transformingBody.skeleton.joints[13].position.xyz.y;
          transformedCustomBody.elbow_right_z = transformingBody.skeleton.joints[13].position.xyz.z;
          transformedCustomBody.elbow_right_QX = transformingBody.skeleton.joints[13].orientation.wxyz.x;
          transformedCustomBody.elbow_right_QY = transformingBody.skeleton.joints[13].orientation.wxyz.y;
          transformedCustomBody.elbow_right_QZ = transformingBody.skeleton.joints[13].orientation.wxyz.z;
          transformedCustomBody.elbow_right_QW = transformingBody.skeleton.joints[13].orientation.wxyz.w;
          transformedCustomBody.elbow_right_conf = transformingBody.skeleton.joints[13].confidence_level;
          transformedCustomBody.wrist_right_x = transformingBody.skeleton.joints[14].position.xyz.x;
          transformedCustomBody.wrist_right_y = transformingBody.skeleton.joints[14].position.xyz.y;
          transformedCustomBody.wrist_right_z = transformingBody.skeleton.joints[14].position.xyz.z;
          transformedCustomBody.wrist_right_QX = transformingBody.skeleton.joints[14].orientation.wxyz.x;
          transformedCustomBody.wrist_right_QY = transformingBody.skeleton.joints[14].orientation.wxyz.y;
          transformedCustomBody.wrist_right_QZ = transformingBody.skeleton.joints[14].orientation.wxyz.z;
          transformedCustomBody.wrist_right_QW = transformingBody.skeleton.joints[14].orientation.wxyz.w;
          transformedCustomBody.wrist_right_conf = transformingBody.skeleton.joints[14].confidence_level;
          transformedCustomBody.hand_right_x = transformingBody.skeleton.joints[15].position.xyz.x;
          transformedCustomBody.hand_right_y = transformingBody.skeleton.joints[15].position.xyz.y;
          transformedCustomBody.hand_right_z = transformingBody.skeleton.joints[15].position.xyz.z;
          transformedCustomBody.hand_right_QX = transformingBody.skeleton.joints[15].orientation.wxyz.x;
          transformedCustomBody.hand_right_QY = transformingBody.skeleton.joints[15].orientation.wxyz.y;
          transformedCustomBody.hand_right_QZ = transformingBody.skeleton.joints[15].orientation.wxyz.z;
          transformedCustomBody.hand_right_QW = transformingBody.skeleton.joints[15].orientation.wxyz.w;
          transformedCustomBody.hand_right_conf = transformingBody.skeleton.joints[15].confidence_level;
          transformedCustomBody.handtip_right_x = transformingBody.skeleton.joints[16].position.xyz.x;
          transformedCustomBody.handtip_right_y = transformingBody.skeleton.joints[16].position.xyz.y;
          transformedCustomBody.handtip_right_z = transformingBody.skeleton.joints[16].position.xyz.z;
          transformedCustomBody.handtip_right_QX = transformingBody.skeleton.joints[16].orientation.wxyz.x;
          transformedCustomBody.handtip_right_QY = transformingBody.skeleton.joints[16].orientation.wxyz.y;
          transformedCustomBody.handtip_right_QZ = transformingBody.skeleton.joints[16].orientation.wxyz.z;
          transformedCustomBody.handtip_right_QW = transformingBody.skeleton.joints[16].orientation.wxyz.w;
          transformedCustomBody.handtip_right_conf = transformingBody.skeleton.joints[16].confidence_level;
          transformedCustomBody.thumb_right_x = transformingBody.skeleton.joints[17].position.xyz.x;
          transformedCustomBody.thumb_right_y = transformingBody.skeleton.joints[17].position.xyz.y;
          transformedCustomBody.thumb_right_z = transformingBody.skeleton.joints[17].position.xyz.z;
          transformedCustomBody.thumb_right_QX = transformingBody.skeleton.joints[17].orientation.wxyz.x;
          transformedCustomBody.thumb_right_QY = transformingBody.skeleton.joints[17].orientation.wxyz.y;
          transformedCustomBody.thumb_right_QZ = transformingBody.skeleton.joints[17].orientation.wxyz.z;
          transformedCustomBody.thumb_right_QW = transformingBody.skeleton.joints[17].orientation.wxyz.w;
          transformedCustomBody.thumb_right_conf = transformingBody.skeleton.joints[17].confidence_level;
          transformedCustomBody.hip_left_x = transformingBody.skeleton.joints[18].position.xyz.x;
          transformedCustomBody.hip_left_y = transformingBody.skeleton.joints[18].position.xyz.y;
          transformedCustomBody.hip_left_z = transformingBody.skeleton.joints[18].position.xyz.z;
          transformedCustomBody.hip_left_QX = transformingBody.skeleton.joints[18].orientation.wxyz.x;
          transformedCustomBody.hip_left_QY = transformingBody.skeleton.joints[18].orientation.wxyz.y;
          transformedCustomBody.hip_left_QZ = transformingBody.skeleton.joints[18].orientation.wxyz.z;
          transformedCustomBody.hip_left_QW = transformingBody.skeleton.joints[18].orientation.wxyz.w;
          transformedCustomBody.hip_left_conf = transformingBody.skeleton.joints[18].confidence_level;
          transformedCustomBody.knee_left_x = transformingBody.skeleton.joints[19].position.xyz.x;
          transformedCustomBody.knee_left_y = transformingBody.skeleton.joints[19].position.xyz.y;
          transformedCustomBody.knee_left_z = transformingBody.skeleton.joints[19].position.xyz.z;
          transformedCustomBody.knee_left_QX = transformingBody.skeleton.joints[19].orientation.wxyz.x;
          transformedCustomBody.knee_left_QY = transformingBody.skeleton.joints[19].orientation.wxyz.y;
          transformedCustomBody.knee_left_QZ = transformingBody.skeleton.joints[19].orientation.wxyz.z;
          transformedCustomBody.knee_left_QW = transformingBody.skeleton.joints[19].orientation.wxyz.w;
          transformedCustomBody.knee_left_conf = transformingBody.skeleton.joints[19].confidence_level;
          transformedCustomBody.ankle_left_x = transformingBody.skeleton.joints[20].position.xyz.x;
          transformedCustomBody.ankle_left_y = transformingBody.skeleton.joints[20].position.xyz.y;
          transformedCustomBody.ankle_left_z = transformingBody.skeleton.joints[20].position.xyz.z;
          transformedCustomBody.ankle_left_QX = transformingBody.skeleton.joints[20].orientation.wxyz.x;
          transformedCustomBody.ankle_left_QY = transformingBody.skeleton.joints[20].orientation.wxyz.y;
          transformedCustomBody.ankle_left_QZ = transformingBody.skeleton.joints[20].orientation.wxyz.z;
          transformedCustomBody.ankle_left_QW = transformingBody.skeleton.joints[20].orientation.wxyz.w;
          transformedCustomBody.ankle_left_conf = transformingBody.skeleton.joints[20].confidence_level;
          transformedCustomBody.foot_left_x = transformingBody.skeleton.joints[21].position.xyz.x;
          transformedCustomBody.foot_left_y = transformingBody.skeleton.joints[21].position.xyz.y;
          transformedCustomBody.foot_left_z = transformingBody.skeleton.joints[21].position.xyz.z;
          transformedCustomBody.foot_left_QX = transformingBody.skeleton.joints[21].orientation.wxyz.x;
          transformedCustomBody.foot_left_QY = transformingBody.skeleton.joints[21].orientation.wxyz.y;
          transformedCustomBody.foot_left_QZ = transformingBody.skeleton.joints[21].orientation.wxyz.z;
          transformedCustomBody.foot_left_QW = transformingBody.skeleton.joints[21].orientation.wxyz.w;
          transformedCustomBody.foot_left_conf = transformingBody.skeleton.joints[21].confidence_level;
          transformedCustomBody.hip_right_x = transformingBody.skeleton.joints[22].position.xyz.x;
          transformedCustomBody.hip_right_y = transformingBody.skeleton.joints[22].position.xyz.y;
          transformedCustomBody.hip_right_z = transformingBody.skeleton.joints[22].position.xyz.z;
          transformedCustomBody.hip_right_QX = transformingBody.skeleton.joints[22].orientation.wxyz.x;
          transformedCustomBody.hip_right_QY = transformingBody.skeleton.joints[22].orientation.wxyz.y;
          transformedCustomBody.hip_right_QZ = transformingBody.skeleton.joints[22].orientation.wxyz.z;
          transformedCustomBody.hip_right_QW = transformingBody.skeleton.joints[22].orientation.wxyz.w;
          transformedCustomBody.hip_right_conf = transformingBody.skeleton.joints[22].confidence_level;
          transformedCustomBody.knee_right_x = transformingBody.skeleton.joints[23].position.xyz.x;
          transformedCustomBody.knee_right_y = transformingBody.skeleton.joints[23].position.xyz.y;
          transformedCustomBody.knee_right_z = transformingBody.skeleton.joints[23].position.xyz.z;
          transformedCustomBody.knee_right_QX = transformingBody.skeleton.joints[23].orientation.wxyz.x;
          transformedCustomBody.knee_right_QY = transformingBody.skeleton.joints[23].orientation.wxyz.y;
          transformedCustomBody.knee_right_QZ = transformingBody.skeleton.joints[23].orientation.wxyz.z;
          transformedCustomBody.knee_right_QW = transformingBody.skeleton.joints[23].orientation.wxyz.w;
          transformedCustomBody.knee_right_conf = transformingBody.skeleton.joints[23].confidence_level;
          transformedCustomBody.ankle_right_x = transformingBody.skeleton.joints[24].position.xyz.x;
          transformedCustomBody.ankle_right_y = transformingBody.skeleton.joints[24].position.xyz.y;
          transformedCustomBody.ankle_right_z = transformingBody.skeleton.joints[24].position.xyz.z;
          transformedCustomBody.ankle_right_QX = transformingBody.skeleton.joints[24].orientation.wxyz.x;
          transformedCustomBody.ankle_right_QY = transformingBody.skeleton.joints[24].orientation.wxyz.y;
          transformedCustomBody.ankle_right_QZ = transformingBody.skeleton.joints[24].orientation.wxyz.z;
          transformedCustomBody.ankle_right_QW = transformingBody.skeleton.joints[24].orientation.wxyz.w;
          transformedCustomBody.ankle_right_conf = transformingBody.skeleton.joints[24].confidence_level;
          transformedCustomBody.foot_right_x = transformingBody.skeleton.joints[25].position.xyz.x;
          transformedCustomBody.foot_right_y = transformingBody.skeleton.joints[25].position.xyz.y;
          transformedCustomBody.foot_right_z = transformingBody.skeleton.joints[25].position.xyz.z;
          transformedCustomBody.foot_right_QX = transformingBody.skeleton.joints[25].orientation.wxyz.x;
          transformedCustomBody.foot_right_QY = transformingBody.skeleton.joints[25].orientation.wxyz.y;
          transformedCustomBody.foot_right_QZ = transformingBody.skeleton.joints[25].orientation.wxyz.z;
          transformedCustomBody.foot_right_QW = transformingBody.skeleton.joints[25].orientation.wxyz.w;
          transformedCustomBody.foot_right_conf = transformingBody.skeleton.joints[25].confidence_level;
          transformedCustomBody.head_x = transformingBody.skeleton.joints[26].position.xyz.x;
          transformedCustomBody.head_y = transformingBody.skeleton.joints[26].position.xyz.y;
          transformedCustomBody.head_z = transformingBody.skeleton.joints[26].position.xyz.z;
          transformedCustomBody.head_QX = transformingBody.skeleton.joints[26].orientation.wxyz.x;
          transformedCustomBody.head_QY = transformingBody.skeleton.joints[26].orientation.wxyz.y;
          transformedCustomBody.head_QZ = transformingBody.skeleton.joints[26].orientation.wxyz.z;
          transformedCustomBody.head_QW = transformingBody.skeleton.joints[26].orientation.wxyz.w;
          transformedCustomBody.head_conf = transformingBody.skeleton.joints[26].confidence_level;
          transformedCustomBody.nose_x = transformingBody.skeleton.joints[27].position.xyz.x;
          transformedCustomBody.nose_y = transformingBody.skeleton.joints[27].position.xyz.y;
          transformedCustomBody.nose_z = transformingBody.skeleton.joints[27].position.xyz.z;
          transformedCustomBody.nose_QX = transformingBody.skeleton.joints[27].orientation.wxyz.x;
          transformedCustomBody.nose_QY = transformingBody.skeleton.joints[27].orientation.wxyz.y;
          transformedCustomBody.nose_QZ = transformingBody.skeleton.joints[27].orientation.wxyz.z;
          transformedCustomBody.nose_QW = transformingBody.skeleton.joints[27].orientation.wxyz.w;
          transformedCustomBody.nose_conf = transformingBody.skeleton.joints[27].confidence_level;
          transformedCustomBody.eye_left_x = transformingBody.skeleton.joints[28].position.xyz.x;
          transformedCustomBody.eye_left_y = transformingBody.skeleton.joints[28].position.xyz.y;
          transformedCustomBody.eye_left_z = transformingBody.skeleton.joints[28].position.xyz.z;
          transformedCustomBody.eye_left_QX = transformingBody.skeleton.joints[28].orientation.wxyz.x;
          transformedCustomBody.eye_left_QY = transformingBody.skeleton.joints[28].orientation.wxyz.y;
          transformedCustomBody.eye_left_QZ = transformingBody.skeleton.joints[28].orientation.wxyz.z;
          transformedCustomBody.eye_left_QW = transformingBody.skeleton.joints[28].orientation.wxyz.w;
          transformedCustomBody.eye_left_conf = transformingBody.skeleton.joints[28].confidence_level;
          transformedCustomBody.ear_left_x = transformingBody.skeleton.joints[29].position.xyz.x;
          transformedCustomBody.ear_left_y = transformingBody.skeleton.joints[29].position.xyz.y;
          transformedCustomBody.ear_left_z = transformingBody.skeleton.joints[29].position.xyz.z;
          transformedCustomBody.ear_left_QX = transformingBody.skeleton.joints[29].orientation.wxyz.x;
          transformedCustomBody.ear_left_QY = transformingBody.skeleton.joints[29].orientation.wxyz.y;
          transformedCustomBody.ear_left_QZ = transformingBody.skeleton.joints[29].orientation.wxyz.z;
          transformedCustomBody.ear_left_QW = transformingBody.skeleton.joints[29].orientation.wxyz.w;
          transformedCustomBody.ear_left_conf = transformingBody.skeleton.joints[29].confidence_level;
          transformedCustomBody.eye_right_x = transformingBody.skeleton.joints[30].position.xyz.x;
          transformedCustomBody.eye_right_y = transformingBody.skeleton.joints[30].position.xyz.y;
          transformedCustomBody.eye_right_z = transformingBody.skeleton.joints[30].position.xyz.z;
          transformedCustomBody.eye_right_QX = transformingBody.skeleton.joints[30].orientation.wxyz.x;
          transformedCustomBody.eye_right_QY = transformingBody.skeleton.joints[30].orientation.wxyz.y;
          transformedCustomBody.eye_right_QZ = transformingBody.skeleton.joints[30].orientation.wxyz.z;
          transformedCustomBody.eye_right_QW = transformingBody.skeleton.joints[30].orientation.wxyz.w;
          transformedCustomBody.eye_right_conf = transformingBody.skeleton.joints[30].confidence_level;
          transformedCustomBody.ear_right_x = transformingBody.skeleton.joints[31].position.xyz.x;
          transformedCustomBody.ear_right_y = transformingBody.skeleton.joints[31].position.xyz.y;
          transformedCustomBody.ear_right_z = transformingBody.skeleton.joints[31].position.xyz.z;
          transformedCustomBody.ear_right_QX = transformingBody.skeleton.joints[31].orientation.wxyz.x;
          transformedCustomBody.ear_right_QY = transformingBody.skeleton.joints[31].orientation.wxyz.y;
          transformedCustomBody.ear_right_QZ = transformingBody.skeleton.joints[31].orientation.wxyz.z;
          transformedCustomBody.ear_right_QW = transformingBody.skeleton.joints[31].orientation.wxyz.w;
          transformedCustomBody.ear_right_conf = transformingBody.skeleton.joints[31].confidence_level;

          m_customBodies[i] = transformedCustomBody;

        }
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
  custom_k4abt_body_t customBody = m_customBodies[n];
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
