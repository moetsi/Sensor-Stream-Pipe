#pragma once

// std
#include <vector>

// libraries
#include <nlohmann/json.hpp>

// project
#include "DetectionNetworkProperties.hpp"
#include "depthai-shared/common/DetectionNetworkType.hpp"
#include "depthai-shared/common/optional.hpp"
#include "depthai-shared/datatype/RawSpatialLocationCalculatorConfig.hpp"

namespace dai {

/**
 * Specify properties for SpatialDetectionNetwork
 */
struct SpatialDetectionNetworkProperties : DetectionNetworkProperties {
    float detectedBBScaleFactor = 1.0;
    SpatialLocationCalculatorConfigThresholds depthThresholds;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SpatialDetectionNetworkProperties,
                                   nnFamily,
                                   blobSize,
                                   blobUri,
                                   numFrames,
                                   numThreads,
                                   numNCEPerThread,
                                   confidenceThreshold,
                                   classes,
                                   coordinates,
                                   anchors,
                                   anchorMasks,
                                   iouThreshold,
                                   detectedBBScaleFactor,
                                   depthThresholds)

}  // namespace dai
