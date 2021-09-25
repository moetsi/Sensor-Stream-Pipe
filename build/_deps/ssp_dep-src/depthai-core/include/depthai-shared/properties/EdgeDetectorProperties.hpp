#pragma once

#include <depthai-shared/common/optional.hpp>
#include <depthai-shared/datatype/RawEdgeDetectorConfig.hpp>
#include <nlohmann/json.hpp>
#include <vector>

namespace dai {

/**
 * Specify properties for EdgeDetector
 */
struct EdgeDetectorProperties {
    /// Initial edge detector config
    RawEdgeDetectorConfig initialConfig;

    /// Whether to wait for config at 'inputConfig' IO
    bool inputConfigSync = false;

    /**
     * Maximum output frame size in bytes (eg: 300x300 BGR image -> 300*300*3 bytes)
     */
    int outputFrameSize = 1 * 1024 * 1024;

    /// Num frames in output pool
    int numFramesPool = 4;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(EdgeDetectorProperties, initialConfig, inputConfigSync, outputFrameSize, numFramesPool);

}  // namespace dai
