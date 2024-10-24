// Copyright (C) 2018-2023 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
//

#pragma once

#include <deque>
#include <iostream>
#include <string>
#include <unordered_map>

#include <opencv2/core.hpp>

///
/// \brief The TrackedObject struct defines properties of detected object.
///
struct TrackedObject {
    cv::Rect rect;  ///< Detected object ROI (zero area if N/A).
    double confidence;  ///< Detection confidence level (-1 if N/A).
    int64_t frame_idx;  ///< Frame index where object was detected (-1 if N/A).
    int object_id;  ///< Unique object identifier (-1 if N/A).
    uint64_t timestamp;  ///< Timestamp in milliseconds.
    cv::Mat fast_descriptor; //this already exists and is computed
    cv::Mat strong_descriptor; //this already exists and is already computed
    float center_x; // this is the x coordinate of the center of the detection
    float center_y; // this is the y coordinate of the center of the detection
    float center_z; // this is the z coordinate of the center of the detection
    int32_t device_id; // this is the device id that made the detection (only needed for global)
    size_t device_id_detection_index; // This is the index position of the detection received by the producer (only needed for global)
    ///
    /// \brief Default constructor.
    ///
    TrackedObject() : confidence(-1), frame_idx(-1), object_id(-1), timestamp(0) {}

    ///
    /// \brief Constructor with parameters.
    /// \param rect Bounding box of detected object.
    /// \param confidence Confidence of detection.
    /// \param frame_idx Index of frame.
    /// \param object_id Object ID.
    ///
    TrackedObject(const cv::Rect& rect, float confidence, int64_t frame_idx, int object_id)
        : rect(rect),
          confidence(confidence),
          frame_idx(frame_idx),
          object_id(object_id),
          timestamp(0) {}
};

using TrackedObjects = std::deque<TrackedObject>;

/// (object id, detected objects) pairs collection.
using ObjectTracks = std::unordered_map<int, TrackedObjects>;
