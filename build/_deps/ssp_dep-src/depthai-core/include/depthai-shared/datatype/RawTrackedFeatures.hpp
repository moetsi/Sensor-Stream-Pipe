#pragma once
#include <cstdint>
#include <nlohmann/json.hpp>
#include <vector>

#include "DatatypeEnum.hpp"
#include "RawBuffer.hpp"
#include "RawFeatureTrackerConfig.hpp"
#include "depthai-shared/common/Point2f.hpp"

namespace dai {

/**
 * TrackedFeature structure
 *
 */
struct TrackedFeature {
    /**
     *  x, y position of the detected feature
     */
    Point2f position;
    /**
     *  Feature ID. Persistent between frames if motion estimation is enabled.
     */
    uint32_t id;
#if 0
    /**
     *  Feature age in frames
     */
    uint32_t age;
    /**
     *  Feature harris score
     */
    float harrisScore;

    /**
     *  Feature tracking error
     */
    float trackingError;
#endif
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TrackedFeature, position, id);

/// RawTrackedFeatures structure
struct RawTrackedFeatures : public RawBuffer {
    std::vector<TrackedFeature> trackedFeatures;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        nlohmann::json j = *this;
        metadata = nlohmann::json::to_msgpack(j);
        datatype = DatatypeEnum::TrackedFeatures;
    };

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(RawTrackedFeatures, trackedFeatures);
};

}  // namespace dai
