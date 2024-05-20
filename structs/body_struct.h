/**
 * \file body_struct.h @brief body struct definition. "Body" frame data type.
 */

#pragma once

#include <iterator>
#include <vector>
#include <cstdint>

#ifndef __MOETSI_RAAS__
#include <cereal/archives/binary.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/vector.hpp>
#endif // !__MOETSI_RAAS__

namespace moetsi::ssp {

#ifndef SWIG 

inline void inplace_hton(uint32_t & h) {
  uint32_t idx = 0x00010203;
  uint32_t dest = 0x0;
  uint8_t *ip = (uint8_t *)&idx;
  uint8_t *sp = (uint8_t *)&h;
  uint8_t *dp = (uint8_t *)&dest;
  for (int i=0; i<4; ++i) {
    int k = ip[i];
    dp[i] = sp[k];
  }
  h = dest;
}

inline void inplace_hton(int32_t &h) {
  uint32_t &h2 = *((uint32_t *)&h);
  inplace_hton(h2);
}

inline void inplace_ntoh(uint32_t &h) {
  inplace_hton(h);
}

inline void inplace_ntoh(int32_t &h) {
  inplace_hton(h);
}

#endif


struct coco_human_t
{
  inline void ntoh() {
    inplace_ntoh(Id);
  }

  inline void hton() {
    inplace_hton(Id);
  }

  int32_t Id;
  float neck_x;
  float neck_y;
  float neck_z;
  float neck_QX;
  float neck_QY;
  float neck_QZ;
  float neck_QW;
  float neck_conf;
  float nose_x;
  float nose_y;
  float nose_z;
  float nose_QX;
  float nose_QY;
  float nose_QZ;
  float nose_QW;
  float nose_conf;
  float pelvis_x;
  float pelvis_y;
  float pelvis_z;
  float pelvis_QX;
  float pelvis_QY;
  float pelvis_QZ;
  float pelvis_QW;
  float pelvis_conf;
  float shoulder_left_x;
  float shoulder_left_y;
  float shoulder_left_z;
  float shoulder_left_QX;
  float shoulder_left_QY;
  float shoulder_left_QZ;
  float shoulder_left_QW;
  float shoulder_left_conf;
  float elbow_left_x;
  float elbow_left_y;
  float elbow_left_z;
  float elbow_left_QX;
  float elbow_left_QY;
  float elbow_left_QZ;
  float elbow_left_QW;
  float elbow_left_conf;
  float wrist_left_x;
  float wrist_left_y;
  float wrist_left_z;
  float wrist_left_QX;
  float wrist_left_QY;
  float wrist_left_QZ;
  float wrist_left_QW;
  float wrist_left_conf;
  float hip_left_x;
  float hip_left_y;
  float hip_left_z;
  float hip_left_QX;
  float hip_left_QY;
  float hip_left_QZ;
  float hip_left_QW;
  float hip_left_conf;
  float knee_left_x;
  float knee_left_y;
  float knee_left_z;
  float knee_left_QX;
  float knee_left_QY;
  float knee_left_QZ;
  float knee_left_QW;
  float knee_left_conf;
  float ankle_left_x;
  float ankle_left_y;
  float ankle_left_z;
  float ankle_left_QX;
  float ankle_left_QY;
  float ankle_left_QZ;
  float ankle_left_QW;
  float ankle_left_conf;
  float shoulder_right_x;
  float shoulder_right_y;
  float shoulder_right_z;
  float shoulder_right_QX;
  float shoulder_right_QY;
  float shoulder_right_QZ;
  float shoulder_right_QW;
  float shoulder_right_conf;
  float elbow_right_x;
  float elbow_right_y;
  float elbow_right_z;
  float elbow_right_QX;
  float elbow_right_QY;
  float elbow_right_QZ;
  float elbow_right_QW;
  float elbow_right_conf;
  float wrist_right_x;
  float wrist_right_y;
  float wrist_right_z;
  float wrist_right_QX;
  float wrist_right_QY;
  float wrist_right_QZ;
  float wrist_right_QW;
  float wrist_right_conf;  
  float hip_right_x;
  float hip_right_y;
  float hip_right_z;
  float hip_right_QX;
  float hip_right_QY;
  float hip_right_QZ;
  float hip_right_QW;
  float hip_right_conf;
  float knee_right_x;
  float knee_right_y;
  float knee_right_z;
  float knee_right_QX;
  float knee_right_QY;
  float knee_right_QZ;
  float knee_right_QW;
  float knee_right_conf;
  float ankle_right_x;
  float ankle_right_y;
  float ankle_right_z;
  float ankle_right_QX;
  float ankle_right_QY;
  float ankle_right_QZ;
  float ankle_right_QW;
  float ankle_right_conf;
  float eye_left_x;
  float eye_left_y;
  float eye_left_z;
  float eye_left_QX;
  float eye_left_QY;
  float eye_left_QZ;
  float eye_left_QW;
  float eye_left_conf;
  float ear_left_x;
  float ear_left_y;
  float ear_left_z;
  float ear_left_QX;
  float ear_left_QY;
  float ear_left_QZ;
  float ear_left_QW;
  float ear_left_conf;
  float eye_right_x;
  float eye_right_y;
  float eye_right_z;
  float eye_right_QX;
  float eye_right_QY;
  float eye_right_QZ;
  float eye_right_QW;
  float eye_right_conf;
  float ear_right_x;
  float ear_right_y;
  float ear_right_z;
  float ear_right_QX;
  float ear_right_QY;
  float ear_right_QZ;
  float ear_right_QW;
  float ear_right_conf;
// TODO FIXME htons!

  float neck_2d_conf;
  float nose_2d_conf;
  float pelvis_2d_conf;
  float shoulder_left_2d_conf;
  float elbow_left_2d_conf;
  float wrist_left_2d_conf;
  float hip_left_2d_conf;
  float knee_left_2d_conf;
  float ankle_left_2d_conf;
  float shoulder_right_2d_conf;
  float elbow_right_2d_conf;
  float wrist_right_2d_conf;
  float hip_right_2d_conf;
  float knee_right_2d_conf;
  float ankle_right_2d_conf;
  float eye_left_2d_conf;
  float ear_left_2d_conf;
  float eye_right_2d_conf;
  float ear_right_2d_conf;
  
  int16_t neck_2d_x;
  int16_t neck_2d_y;
  int16_t neck_2d_depth;

  int16_t nose_2d_x;
  int16_t nose_2d_y;
  int16_t nose_2d_depth;

  int16_t pelvis_2d_x;
  int16_t pelvis_2d_y;
  int16_t pelvis_2d_depth;

  int16_t shoulder_left_2d_x;
  int16_t shoulder_left_2d_y;
  int16_t shoulder_left_2d_depth;

  int16_t elbow_left_2d_x;
  int16_t elbow_left_2d_y;
  int16_t elbow_left_2d_depth;

  int16_t wrist_left_2d_x;
  int16_t wrist_left_2d_y;
  int16_t wrist_left_2d_depth;

  int16_t hip_left_2d_x;
  int16_t hip_left_2d_y;
  int16_t hip_left_2d_depth;

  int16_t knee_left_2d_x;
  int16_t knee_left_2d_y;
  int16_t knee_left_2d_depth;

  int16_t ankle_left_2d_x;
  int16_t ankle_left_2d_y;
  int16_t ankle_left_2d_depth;

  int16_t shoulder_right_2d_x;
  int16_t shoulder_right_2d_y;
  int16_t shoulder_right_2d_depth;

  int16_t elbow_right_2d_x;
  int16_t elbow_right_2d_y;
  int16_t elbow_right_2d_depth;

  int16_t wrist_right_2d_x;
  int16_t wrist_right_2d_y;
  int16_t wrist_right_2d_depth;

  int16_t hip_right_2d_x;
  int16_t hip_right_2d_y;
  int16_t hip_right_2d_depth;

  int16_t knee_right_2d_x;
  int16_t knee_right_2d_y;
  int16_t knee_right_2d_depth;

  int16_t ankle_right_2d_x;
  int16_t ankle_right_2d_y;
  int16_t ankle_right_2d_depth;

  int16_t eye_left_2d_x;
  int16_t eye_left_2d_y;
  int16_t eye_left_2d_depth;

  int16_t ear_left_2d_x;
  int16_t ear_left_2d_y;
  int16_t ear_left_2d_depth;

  int16_t eye_right_2d_x;
  int16_t eye_right_2d_y;
  int16_t eye_right_2d_depth;

  int16_t ear_right_2d_x;
  int16_t ear_right_2d_y;
  int16_t ear_right_2d_depth;

  
#ifndef __MOETSI_RAAS__
  // Serialize method (not used by Server but is available)
  template <class Archive> void serialize(Archive &ar) {
    ar(
      Id,
      neck_x,
      neck_y,
      neck_z,
      neck_QX,
      neck_QY,
      neck_QZ,
      neck_QW,
      neck_conf,
      nose_x,
      nose_y,
      nose_z,
      nose_QX,
      nose_QY,
      nose_QZ,
      nose_QW,
      nose_conf,
      pelvis_x,
      pelvis_y,
      pelvis_z,
      pelvis_QX,
      pelvis_QY,
      pelvis_QZ,
      pelvis_QW,
      pelvis_conf,
      shoulder_left_x,
      shoulder_left_y,
      shoulder_left_z,
      shoulder_left_QX,
      shoulder_left_QY,
      shoulder_left_QZ,
      shoulder_left_QW,
      shoulder_left_conf,
      elbow_left_x,
      elbow_left_y,
      elbow_left_z,
      elbow_left_QX,
      elbow_left_QY,
      elbow_left_QZ,
      elbow_left_QW,
      elbow_left_conf,
      wrist_left_x,
      wrist_left_y,
      wrist_left_z,
      wrist_left_QX,
      wrist_left_QY,
      wrist_left_QZ,
      wrist_left_QW,
      wrist_left_conf,
      hip_left_x,
      hip_left_y,
      hip_left_z,
      hip_left_QX,
      hip_left_QY,
      hip_left_QZ,
      hip_left_QW,
      hip_left_conf,
      knee_left_x,
      knee_left_y,
      knee_left_z,
      knee_left_QX,
      knee_left_QY,
      knee_left_QZ,
      knee_left_QW,
      knee_left_conf,
      ankle_left_x,
      ankle_left_y,
      ankle_left_z,
      ankle_left_QX,
      ankle_left_QY,
      ankle_left_QZ,
      ankle_left_QW,
      ankle_left_conf,
      shoulder_right_x,
      shoulder_right_y,
      shoulder_right_z,
      shoulder_right_QX,
      shoulder_right_QY,
      shoulder_right_QZ,
      shoulder_right_QW,
      shoulder_right_conf,
      elbow_right_x,
      elbow_right_y,
      elbow_right_z,
      elbow_right_QX,
      elbow_right_QY,
      elbow_right_QZ,
      elbow_right_QW,
      elbow_right_conf,
      wrist_right_x,
      wrist_right_y,
      wrist_right_z,
      wrist_right_QX,
      wrist_right_QY,
      wrist_right_QZ,
      wrist_right_QW,
      wrist_right_conf,  
      hip_right_x,
      hip_right_y,
      hip_right_z,
      hip_right_QX,
      hip_right_QY,
      hip_right_QZ,
      hip_right_QW,
      hip_right_conf,
      knee_right_x,
      knee_right_y,
      knee_right_z,
      knee_right_QX,
      knee_right_QY,
      knee_right_QZ,
      knee_right_QW,
      knee_right_conf,
      ankle_right_x,
      ankle_right_y,
      ankle_right_z,
      ankle_right_QX,
      ankle_right_QY,
      ankle_right_QZ,
      ankle_right_QW,
      ankle_right_conf,
      eye_left_x,
      eye_left_y,
      eye_left_z,
      eye_left_QX,
      eye_left_QY,
      eye_left_QZ,
      eye_left_QW,
      eye_left_conf,
      ear_left_x,
      ear_left_y,
      ear_left_z,
      ear_left_QX,
      ear_left_QY,
      ear_left_QZ,
      ear_left_QW,
      ear_left_conf,
      eye_right_x,
      eye_right_y,
      eye_right_z,
      eye_right_QX,
      eye_right_QY,
      eye_right_QZ,
      eye_right_QW,
      eye_right_conf,
      ear_right_x,
      ear_right_y,
      ear_right_z,
      ear_right_QX,
      ear_right_QY,
      ear_right_QZ,
      ear_right_QW,
      ear_right_conf,

      neck_2d_x,
      neck_2d_y,
      neck_2d_depth,
      neck_2d_conf,
      nose_2d_x,
      nose_2d_y,
      nose_2d_depth,
      nose_2d_conf,
      pelvis_2d_x,
      pelvis_2d_y,
      pelvis_2d_depth,
      pelvis_2d_conf,
      shoulder_left_2d_x,
      shoulder_left_2d_y,
      shoulder_left_2d_depth,
      shoulder_left_2d_conf,
      elbow_left_2d_x,
      elbow_left_2d_y,
      elbow_left_2d_depth,
      elbow_left_2d_conf,
      wrist_left_2d_x,
      wrist_left_2d_y,
      wrist_left_2d_depth,
      wrist_left_2d_conf,
      hip_left_2d_x,
      hip_left_2d_y,
      hip_left_2d_depth,
      hip_left_2d_conf,
      knee_left_2d_x,
      knee_left_2d_y,
      knee_left_2d_depth,
      knee_left_2d_conf,
      ankle_left_2d_x,
      ankle_left_2d_y,
      ankle_left_2d_depth,
      ankle_left_2d_conf,
      shoulder_right_2d_x,
      shoulder_right_2d_y,
      shoulder_right_2d_depth,
      shoulder_right_2d_conf,
      elbow_right_2d_x,
      elbow_right_2d_y,
      elbow_right_2d_depth,
      elbow_right_2d_conf,
      wrist_right_2d_x,
      wrist_right_2d_y,
      wrist_right_2d_depth,
      wrist_right_2d_conf,
      hip_right_2d_x,
      hip_right_2d_y,
      hip_right_2d_depth,
      hip_right_2d_conf,
      knee_right_2d_x,
      knee_right_2d_y,
      knee_right_2d_depth,
      knee_right_2d_conf,
      ankle_right_2d_x,
      ankle_right_2d_y,
      ankle_right_2d_depth,
      ankle_right_2d_conf,
      eye_left_2d_x,
      eye_left_2d_y,
      eye_left_2d_depth,
      eye_left_2d_conf,
      ear_left_2d_x,
      ear_left_2d_y,
      ear_left_2d_depth,
      ear_left_2d_conf,
      eye_right_2d_x,
      eye_right_2d_y,
      eye_right_2d_depth,
      eye_right_2d_conf,
      ear_right_2d_x,
      ear_right_2d_y,
      ear_right_2d_depth,
      ear_right_2d_conf

    );
  }
#endif // !__MOETSI_RAAS__

};

} // namespace moetsi::ssp
