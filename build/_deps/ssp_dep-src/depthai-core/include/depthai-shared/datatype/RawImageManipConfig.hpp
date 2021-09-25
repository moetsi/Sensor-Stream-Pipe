#pragma once
#include <cstdint>
#include <nlohmann/json.hpp>
#include <vector>

#include "DatatypeEnum.hpp"
#include "RawBuffer.hpp"
#include "RawImgFrame.hpp"

// shared
#include "depthai-shared/common/Point2f.hpp"
#include "depthai-shared/common/RotatedRect.hpp"
#include "depthai-shared/common/Size2f.hpp"

namespace dai {

/// RawImageManipConfig structure
struct RawImageManipConfig : public RawBuffer {
    // NNData data is in PoBuf
    struct CropRect {
        // Normalized range 0-1
        float xmin = 0.0f, ymin = 0.0f, xmax = 0.0f, ymax = 0.0f;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(CropRect, xmin, ymin, xmax, ymax);
    };

    struct CropConfig {
        CropRect cropRect;
        RotatedRect cropRotatedRect;

        bool enableCenterCropRectangle = false;
        // if enableCenterCropRectangle -> automatically calculated crop parameters
        float cropRatio = 1.0f, widthHeightAspectRatio = 1.0f;

        bool enableRotatedRect = false;

        // Range 0..1 by default. Set 'false' to specify in pixels
        bool normalizedCoords = true;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            CropConfig, cropRect, cropRotatedRect, enableCenterCropRectangle, cropRatio, widthHeightAspectRatio, enableRotatedRect, normalizedCoords);
    };

    struct ResizeConfig {
        int width = 0, height = 0;
        bool lockAspectRatioFill = false;
        char bgRed = 0, bgGreen = 0, bgBlue = 0;

        //  clockwise order, pt[0] is mapped to the top-left output corner
        std::vector<Point2f> warpFourPoints;
        bool normalizedCoords = true;
        bool enableWarp4pt = false;

        std::vector<float> warpMatrix3x3;
        bool enableWarpMatrix = false;

        // Warp background / border mode: replicates pixels if true,
        // otherwise fills with a constant color defined by: bgRed, bgGreen, bgBlue
        bool warpBorderReplicate = false;

        // clockwise
        float rotationAngleDeg;
        bool enableRotation = false;

        /**
         * Whether to keep aspect ratio of input or not
         */
        bool keepAspectRatio = true;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(ResizeConfig,
                                       width,
                                       height,
                                       lockAspectRatioFill,
                                       bgRed,
                                       bgGreen,
                                       bgBlue,
                                       warpFourPoints,
                                       normalizedCoords,
                                       enableWarp4pt,
                                       warpMatrix3x3,
                                       enableWarpMatrix,
                                       warpBorderReplicate,
                                       rotationAngleDeg,
                                       enableRotation,
                                       keepAspectRatio);
    };

    struct FormatConfig {
        RawImgFrame::Type type = RawImgFrame::Type::RGB888p;
        bool flipHorizontal = false;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(FormatConfig, type, flipHorizontal);
    };

    CropConfig cropConfig;
    ResizeConfig resizeConfig;
    FormatConfig formatConfig;

    bool enableCrop = false;
    bool enableResize = false;
    bool enableFormat = false;

    // Usable with runtime config only,
    // when ImageManipProperties.inputConfigSync is set
    bool reusePreviousImage = false;
    bool skipCurrentImage = false;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        nlohmann::json j = *this;
        metadata = nlohmann::json::to_msgpack(j);
        datatype = DatatypeEnum::ImageManipConfig;
    };

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(
        RawImageManipConfig, cropConfig, resizeConfig, formatConfig, enableCrop, enableResize, enableFormat, reusePreviousImage, skipCurrentImage);
};

}  // namespace dai
