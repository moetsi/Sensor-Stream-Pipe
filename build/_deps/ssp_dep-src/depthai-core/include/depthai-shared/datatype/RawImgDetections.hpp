#pragma once

#include "RawBuffer.hpp"
#include "depthai-shared/common/Point3f.hpp"

namespace dai {

/// ImgDetection structure
struct ImgDetection {
    uint32_t label;
    float confidence;
    float xmin;
    float ymin;
    float xmax;
    float ymax;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(ImgDetection, label, confidence, xmin, ymin, xmax, ymax);
};

/// RawImgDetections structure
struct RawImgDetections : public RawBuffer {
    std::vector<ImgDetection> detections;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        nlohmann::json j = *this;
        metadata = nlohmann::json::to_msgpack(j);
        datatype = DatatypeEnum::ImgDetections;
    };

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(RawImgDetections, detections);
};

}  // namespace dai
