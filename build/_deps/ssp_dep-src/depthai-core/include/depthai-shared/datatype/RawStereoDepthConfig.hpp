#pragma once
#include <cstdint>
#include <nlohmann/json.hpp>
#include <vector>

#include "DatatypeEnum.hpp"
#include "RawBuffer.hpp"

namespace dai {

/**
 * Median filter config for disparity post-processing
 */
enum class MedianFilter : int32_t { MEDIAN_OFF = 0, KERNEL_3x3 = 3, KERNEL_5x5 = 5, KERNEL_7x7 = 7 };

/// StereoDepth configuration data structure
struct StereoDepthConfigData {
    using MedianFilter = dai::MedianFilter;

    /**
     * Set kernel size for disparity/depth median filtering, or disable
     */
    MedianFilter median = MedianFilter::KERNEL_5x5;

    /**
     * Confidence threshold for disparity calculation, 0..255
     */
    std::int32_t confidenceThreshold = 230;

    /**
     * Sigma value for bilateral filter. 0 means disabled
     * A larger value of the parameter means that farther colors within the pixel neighborhood will be mixed together.
     */
    std::int16_t bilateralSigmaValue = 0;

    /**
     * Left-right check threshold for left-right, right-left disparity map combine, 0..128
     * Used only when left-right check mode is enabled.
     * Defines the maximum difference between the confidence of pixels from left-right and right-left confidence maps
     */
    std::int32_t leftRightCheckThreshold = 4;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(StereoDepthConfigData, median, confidenceThreshold, bilateralSigmaValue, leftRightCheckThreshold);

/// RawStereoDepthConfig configuration structure
struct RawStereoDepthConfig : public RawBuffer {
    StereoDepthConfigData config;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        nlohmann::json j = *this;
        metadata = nlohmann::json::to_msgpack(j);
        datatype = DatatypeEnum::StereoDepthConfig;
    };

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(RawStereoDepthConfig, config);
};

}  // namespace dai
