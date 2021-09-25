#pragma once

#include <depthai-shared/common/optional.hpp>
#include <depthai-shared/datatype/RawSpatialLocationCalculatorConfig.hpp>
#include <nlohmann/json.hpp>
#include <vector>

namespace dai {

/**
 * Specify properties for SpatialLocationCalculator
 */
struct SpatialLocationCalculatorProperties {
    RawSpatialLocationCalculatorConfig roiConfig;

    /// Whether to wait for config at 'inputConfig' IO
    bool inputConfigSync = false;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SpatialLocationCalculatorProperties, roiConfig, inputConfigSync);

}  // namespace dai
