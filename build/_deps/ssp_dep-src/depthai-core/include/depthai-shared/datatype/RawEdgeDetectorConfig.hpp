#pragma once
#include <cstdint>
#include <nlohmann/json.hpp>
#include <vector>

#include "DatatypeEnum.hpp"
#include "RawBuffer.hpp"
#include "RawImgFrame.hpp"
#include "depthai-shared/common/Rect.hpp"

namespace dai {

/// EdgeDetectorConfigData configuration data structure
struct EdgeDetectorConfigData {
    /**
     * Used for horizontal gradiant computation in 3x3 Sobel filter
     * Format: 3x3 matrix, 2nd column must be 0
     * Default: +1 0 -1; +2 0 -2; +1 0 -1
     */
    std::vector<std::vector<int>> sobelFilterHorizontalKernel;
    /**
     * Used for vertical gradiant computation in 3x3 Sobel filter
     * Format: 3x3 matrix, 2nd row must be 0
     * Default: +1 +2 +1; 0 0 0; -1 -2 -1
     */
    std::vector<std::vector<int>> sobelFilterVerticalKernel;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(EdgeDetectorConfigData, sobelFilterHorizontalKernel, sobelFilterVerticalKernel);

/// RawEdgeDetectorConfig configuration structure
struct RawEdgeDetectorConfig : public RawBuffer {
    EdgeDetectorConfigData config;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        nlohmann::json j = *this;
        metadata = nlohmann::json::to_msgpack(j);
        datatype = DatatypeEnum::EdgeDetectorConfig;
    };

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(RawEdgeDetectorConfig, config);
};

}  // namespace dai
