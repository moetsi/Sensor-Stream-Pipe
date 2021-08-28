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

#include "../utils/utils.h"

namespace moetsi::ssp {

struct object_human_t
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

#ifndef __MOETSI_RAAS__
  // Serialize method (not used by Server but is available)
  template <class Archive> void serialize(Archive &ar) {
    ar(
      Id,
      pelvis_x,
      pelvis_y,
      pelvis_z,
      pelvis_QX,
      pelvis_QY,
      pelvis_QZ,
      pelvis_QW,
      pelvis_conf,
      spine_navel_x,
      spine_navel_y,
      spine_navel_z,
      spine_navel_QX,
      spine_navel_QY,
      spine_navel_QZ,
      spine_navel_QW,
      spine_navel_conf,
      spine_chest_x,
      spine_chest_y,
      spine_chest_z,
      spine_chest_QX,
      spine_chest_QY,
      spine_chest_QZ,
      spine_chest_QW,
      spine_chest_conf,
      neck_x,
      neck_y,
      neck_z,
      neck_QX,
      neck_QY,
      neck_QZ,
      neck_QW,
      neck_conf,
      clavicle_left_x,
      clavicle_left_y,
      clavicle_left_z,
      clavicle_left_QX,
      clavicle_left_QY,
      clavicle_left_QZ,
      clavicle_left_QW,
      clavicle_left_conf,
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
      hand_left_x,
      hand_left_y,
      hand_left_z,
      hand_left_QX,
      hand_left_QY,
      hand_left_QZ,
      hand_left_QW,
      hand_left_conf,
      handtip_left_x,
      handtip_left_y,
      handtip_left_z,
      handtip_left_QX,
      handtip_left_QY,
      handtip_left_QZ,
      handtip_left_QW,
      handtip_left_conf,
      thumb_left_x,
      thumb_left_y,
      thumb_left_z,
      thumb_left_QX,
      thumb_left_QY,
      thumb_left_QZ,
      thumb_left_QW,
      thumb_left_conf,
      clavicle_right_x,
      clavicle_right_y,
      clavicle_right_z,
      clavicle_right_QX,
      clavicle_right_QY,
      clavicle_right_QZ,
      clavicle_right_QW,
      clavicle_right_conf,
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
      hand_right_x,
      hand_right_y,
      hand_right_z,
      hand_right_QX,
      hand_right_QY,
      hand_right_QZ,
      hand_right_QW,
      hand_right_conf,
      handtip_right_x,
      handtip_right_y,
      handtip_right_z,
      handtip_right_QX,
      handtip_right_QY,
      handtip_right_QZ,
      handtip_right_QW,
      handtip_right_conf,
      thumb_right_x,
      thumb_right_y,
      thumb_right_z,
      thumb_right_QX,
      thumb_right_QY,
      thumb_right_QZ,
      thumb_right_QW,
      thumb_right_conf,
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
      foot_left_x,
      foot_left_y,
      foot_left_z,
      foot_left_QX,
      foot_left_QY,
      foot_left_QZ,
      foot_left_QW,
      foot_left_conf,
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
      foot_right_x,
      foot_right_y,
      foot_right_z,
      foot_right_QX,
      foot_right_QY,
      foot_right_QZ,
      foot_right_QW,
      foot_right_conf,
      head_x,
      head_y,
      head_z,
      head_QX,
      head_QY,
      head_QZ,
      head_QW,
      head_conf,
      nose_x,
      nose_y,
      nose_z,
      nose_QX,
      nose_QY,
      nose_QZ,
      nose_QW,
      nose_conf,
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
      ear_right_conf
    );
  }
#endif // !__MOETSI_RAAS__

};

} // namespace moetsi::ssp
