#pragma once

// std
#include <vector>

// libraries
#include <nlohmann/json.hpp>

// project

#include "depthai-shared/common/optional.hpp"

namespace dai {

enum class TrackerType : std::int32_t {
    /// Ability to track the objects without accessing image data.
    ZERO_TERM_IMAGELESS = 5,
    /// Tracking using image data too.
    ZERO_TERM_COLOR_HISTOGRAM
};

enum class TrackerIdAssigmentPolicy : std::int32_t {
    /// Always take a new, unique ID
    UNIQUE_ID,
    /// Take the smallest available ID
    SMALLEST_ID
};

/**
 * Specify properties for ObjectTracker
 */
struct ObjectTrackerProperties {
    float trackerThreshold = 0.0;
    std::int32_t maxObjectsToTrack = 60;
    std::vector<std::uint32_t> detectionLabelsToTrack;
    TrackerType trackerType = TrackerType::ZERO_TERM_IMAGELESS;
    TrackerIdAssigmentPolicy trackerIdAssigmentPolicy = TrackerIdAssigmentPolicy::UNIQUE_ID;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ObjectTrackerProperties, trackerThreshold, maxObjectsToTrack, detectionLabelsToTrack, trackerType, trackerIdAssigmentPolicy)

}  // namespace dai
