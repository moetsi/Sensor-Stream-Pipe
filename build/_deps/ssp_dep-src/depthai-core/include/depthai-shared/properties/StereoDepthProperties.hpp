#pragma once

#include <depthai-shared/common/EepromData.hpp>
#include <depthai-shared/common/optional.hpp>
#include <nlohmann/json.hpp>

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/datatype/RawStereoDepthConfig.hpp"

namespace dai {

/**
 * Specify properties for StereoDepth
 */
struct StereoDepthProperties {
    struct RectificationMesh {
        /**
         * Uri which points to the mesh array for 'left' input rectification
         */
        std::string meshLeftUri;
        /**
         * Uri which points to the mesh array for 'right' input rectification
         */
        std::string meshRightUri;
        /**
         * Mesh array size in bytes, for each of 'left' and 'right' (need to match)
         */
        tl::optional<std::uint32_t> meshSize;
        /**
         * Distance between mesh points, in the horizontal direction
         */
        uint16_t stepWidth = 16;
        /**
         * Distance between mesh points, in the vertical direction
         */
        uint16_t stepHeight = 16;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(RectificationMesh, meshLeftUri, meshRightUri, meshSize, stepWidth, stepHeight);
    };

    /// Initial stereo config
    RawStereoDepthConfig initialConfig;

    /// Whether to wait for config at 'inputConfig' IO
    bool inputConfigSync = false;

    using MedianFilter = dai::MedianFilter;

    /**
     * Align the disparity/depth to the perspective of a rectified output, or center it
     */
    enum class DepthAlign : int32_t { RECTIFIED_RIGHT, RECTIFIED_LEFT, CENTER };

    /**
     * Calibration data byte array
     */
    std::vector<std::uint8_t> calibration;

    EepromData calibrationData;

    /**
     * Set the disparity/depth alignment to the perspective of a rectified output, or center it
     */
    DepthAlign depthAlign = DepthAlign::RECTIFIED_RIGHT;
    /**
     * Which camera to align disparity/depth to.
     * When configured (not AUTO), takes precedence over 'depthAlign'
     */
    CameraBoardSocket depthAlignCamera = CameraBoardSocket::AUTO;

    bool enableRectification = true;
    /**
     * Computes and combines disparities in both L-R and R-L directions, and combine them.
     * For better occlusion handling
     */
    bool enableLeftRightCheck = false;
    /**
     * Computes disparity with sub-pixel interpolation (5 fractional bits), suitable for long range
     */
    bool enableSubpixel = false;
    /**
     * Disparity range increased from 96 to 192, combined from full resolution and downscaled images.
     * Suitable for short range objects
     */
    bool enableExtendedDisparity = false;
    /**
     * Mirror rectified frames: true to have disparity/depth normal (non-mirrored)
     */
    bool rectifyMirrorFrame = true;
    /**
     * Fill color for missing data at frame edges: grayscale 0..255, or -1 to replicate pixels
     */
    std::int32_t rectifyEdgeFillColor = -1;
    /**
     * Input frame width. Optional (taken from MonoCamera nodes if they exist)
     */
    tl::optional<std::int32_t> width;
    /**
     * Input frame height. Optional (taken from MonoCamera nodes if they exist)
     */
    tl::optional<std::int32_t> height;
    /**
     * Output disparity/depth width. Currently only used when aligning to RGB
     */
    tl::optional<std::int32_t> outWidth;
    /**
     * Output disparity/depth height. Currently only used when aligning to RGB
     */
    tl::optional<std::int32_t> outHeight;
    /**
     * Whether to keep aspect ratio of the input (rectified) or not
     */
    bool outKeepAspectRatio = true;

    /**
     * Specify a direct warp mesh to be used for rectification,
     * instead of intrinsics + extrinsic matrices
     */
    RectificationMesh mesh;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(StereoDepthProperties,
                                   initialConfig,
                                   inputConfigSync,
                                   calibration,
                                   calibrationData,
                                   depthAlign,
                                   depthAlignCamera,
                                   enableRectification,
                                   enableLeftRightCheck,
                                   enableSubpixel,
                                   enableExtendedDisparity,
                                   rectifyMirrorFrame,
                                   rectifyEdgeFillColor,
                                   width,
                                   height,
                                   outWidth,
                                   outHeight,
                                   outKeepAspectRatio,
                                   mesh);

}  // namespace dai
