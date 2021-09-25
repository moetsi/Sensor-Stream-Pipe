#pragma once

#include <iostream>

#include "RawBuffer.hpp"
#include "depthai-shared/common/Point3f.hpp"
#include "depthai-shared/common/Rect.hpp"
#include "depthai-shared/datatype/RawImgDetections.hpp"

namespace dai {

/**
 * Tracklet structure
 *
 * Contains tracklets from object tracker output.
 */
struct Tracklet {
    enum class TrackingStatus : std::int32_t {
        NEW,     /**< The object is newly added. */
        TRACKED, /**< The object is being tracked. */
        LOST,   /**< The object gets lost now. The object can be tracked again automatically(long term tracking) or by specifying detected object manually(short
                  term and zero term tracking). */
        REMOVED /**< The object is removed. */
    };
    /**
     * Tracked region of interest.
     */
    Rect roi;
    /**
     * Tracklet's ID.
     */
    std::int32_t id;
    /**
     * Tracklet's label ID.
     */
    std::int32_t label;
    /**
     * Number of frames it is being tracked for.
     */
    std::int32_t age;
    /**
     * Status of tracklet.
     */
    TrackingStatus status;

    /**
     * Image detection that is tracked.
     */
    ImgDetection srcImgDetection;
    /**
     * Spatial coordinates of tracklet.
     */
    Point3f spatialCoordinates;
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(Tracklet, roi, id, label, age, status, srcImgDetection, spatialCoordinates);
};

/// RawTracklets structure
struct RawTracklets : public RawBuffer {
    std::vector<Tracklet> tracklets;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        nlohmann::json j = *this;
        metadata = nlohmann::json::to_msgpack(j);
        datatype = DatatypeEnum::Tracklets;
    };

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(RawTracklets, tracklets);
};

}  // namespace dai
