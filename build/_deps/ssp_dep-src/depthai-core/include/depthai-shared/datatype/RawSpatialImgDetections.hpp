#pragma once

#include "RawBuffer.hpp"
#include "RawImgDetections.hpp"
#include "depthai-shared/common/Point3f.hpp"

namespace dai {

/**
 * SpatialImgDetection structure
 *
 * Contains image detection results together with spatial location data.
 */
struct SpatialImgDetection : ImgDetection {
    Point3f spatialCoordinates;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(SpatialImgDetection, label, confidence, xmin, ymin, xmax, ymax, spatialCoordinates);
};

/// RawSpatialImgDetections structure
struct RawSpatialImgDetections : public RawBuffer {
    std::vector<SpatialImgDetection> detections;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        nlohmann::json j = *this;
        metadata = nlohmann::json::to_msgpack(j);
        datatype = DatatypeEnum::SpatialImgDetections;
    };

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(RawSpatialImgDetections, detections);
};

}  // namespace dai
