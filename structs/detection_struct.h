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
    int32_t device_id;
    uint64_t device_time;
    int detection_label;
    size_t sensor_track_id;
    float sensor_center_x;
    float sensor_center_y;
    float sensor_center_z;
    size_t global_track_id;
    float global_center_x;
    float global_center_y;
    float global_center_z;
    char sensor_name[128]; // Fixed-size character array for sensor name

    // Constructor to initialize the sensor_name with an empty string
    detection_struct_t() {
        std::memset(sensor_name, 0, sizeof(sensor_name));
    }
};



} // namespace moetsi::ssp
