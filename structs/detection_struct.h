/**
 * \file detction_struct.h @brief detction struct definition. "Detection" frame data type.
 */

#pragma once

#include <iterator>
#include <vector>
#include <cstdint>


namespace moetsi::ssp {

/**
 * @brief Detection struct
 */
struct detection_struct_t
{
  uint64_t device_time;
  size_t track_id;
  float center_x;
  float center_y;
  float center_z;
  int detection_label;
    // 1 = spatial person detection
};


} // namespace moetsi::ssp
