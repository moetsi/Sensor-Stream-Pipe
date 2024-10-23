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
    // Fields to capture cv::Rect data
    int rect_x;
    int rect_y;
    int rect_width;
    int rect_height;
    float confidence;
    float sensor_center_x;
    float sensor_center_y;
    float sensor_center_z;
    size_t device_id_detection_index;
    size_t global_track_id;
    float global_center_x;
    float global_center_y;
    float global_center_z;
    float strong_descriptor[256]; // Fixed-size array for strong descriptor
    char sensor_name[128]; // Fixed-size character array for sensor name

    // Constructor to initialize the sensor_name, strong_descriptor, and rect fields with default values
    detection_struct_t() {
        std::memset(sensor_name, 0, sizeof(sensor_name));
        std::fill(std::begin(strong_descriptor), std::end(strong_descriptor), 0.0f);
        rect_x = 0;
        rect_y = 0;
        rect_width = 0;
        rect_height = 0;
    }
};

} // namespace moetsi::ssp
