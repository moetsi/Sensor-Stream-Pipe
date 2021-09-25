#pragma once

#include <depthai-shared/common/optional.hpp>
#include <depthai-shared/datatype/RawFeatureTrackerConfig.hpp>
#include <nlohmann/json.hpp>
#include <vector>

namespace dai {

/**
 * Specify properties for FeatureTracker
 */
struct FeatureTrackerProperties {
    /**
     * Initial feature tracker config
     */
    RawFeatureTrackerConfig initialConfig;

    /**
     * Whether to wait for config at 'inputConfig' IO
     */
    bool inputConfigSync = false;

    /**
     * Number of shaves reserved for feature tracking.
     * Optical flow can use 1 or 2 shaves, while for corner detection only 1 is enough.
     * Hardware motion estimation doesn't require shaves.
     * Maximum 2, minimum 1.
     */
    std::int32_t numShaves = 1;

    /**
     * Number of memory slices reserved for feature tracking.
     * Optical flow can use 1 or 2 memory slices, while for corner detection only 1 is enough.
     * Maximum number of features depends on the number of allocated memory slices.
     * Hardware motion estimation doesn't require memory slices.
     * Maximum 2, minimum 1.
     */
    std::int32_t numMemorySlices = 1;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(FeatureTrackerProperties, initialConfig, inputConfigSync, numShaves, numMemorySlices);

}  // namespace dai
