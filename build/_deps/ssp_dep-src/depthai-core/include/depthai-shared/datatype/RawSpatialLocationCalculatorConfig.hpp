#pragma once
#include <cstdint>
#include <nlohmann/json.hpp>
#include <vector>

#include "DatatypeEnum.hpp"
#include "RawBuffer.hpp"
#include "RawImgFrame.hpp"
#include "depthai-shared/common/Rect.hpp"

namespace dai {

/**
 * SpatialLocation configuration thresholds structure
 *
 * Contains configuration data for lower and upper threshold in millimeters for ROI.
 * Values outside of threshold range will be ignored when calculating spatial coordinates from depth map.
 */
struct SpatialLocationCalculatorConfigThresholds {
    uint32_t lowerThreshold = 0;
    uint32_t upperThreshold = 65535;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SpatialLocationCalculatorConfigThresholds, lowerThreshold, upperThreshold);

/// SpatialLocation configuration data structure
struct SpatialLocationCalculatorConfigData {
    Rect roi;
    SpatialLocationCalculatorConfigThresholds depthThresholds;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SpatialLocationCalculatorConfigData, roi, depthThresholds);

/// RawSpatialLocation configuration structure
struct RawSpatialLocationCalculatorConfig : public RawBuffer {
    std::vector<SpatialLocationCalculatorConfigData> config;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        nlohmann::json j = *this;
        metadata = nlohmann::json::to_msgpack(j);
        datatype = DatatypeEnum::SpatialLocationCalculatorConfig;
    };

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(RawSpatialLocationCalculatorConfig, config);
};

}  // namespace dai
