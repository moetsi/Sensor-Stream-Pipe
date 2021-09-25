#pragma once

#include <depthai-shared/datatype/RawCameraControl.hpp>
#include <nlohmann/json.hpp>

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/common/CameraImageOrientation.hpp"

namespace dai {

/**
 *  Specify properties for ColorCamera such as camera ID, ...
 */
struct ColorCameraProperties {
    static constexpr int AUTO = -1;

    struct IspScale {
        int32_t horizNumerator = 0;
        int32_t horizDenominator = 0;
        int32_t vertNumerator = 0;
        int32_t vertDenominator = 0;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(IspScale, horizNumerator, horizDenominator, vertNumerator, vertDenominator);
    };

    /**
     * Select the camera sensor resolution
     */
    enum class SensorResolution : int32_t { THE_1080_P, THE_4_K, THE_12_MP };

    /**
     * For 24 bit color these can be either RGB or BGR
     */
    enum class ColorOrder : int32_t { BGR, RGB };

    /*
     * Initial controls applied to ColorCamera node
     */
    RawCameraControl initialControl;

    /**
     * Which socket will color camera use
     */
    CameraBoardSocket boardSocket = CameraBoardSocket::AUTO;

    /**
     * Camera sensor image orientation / pixel readout
     */
    CameraImageOrientation imageOrientation = CameraImageOrientation::AUTO;

    /**
     * For 24 bit color these can be either RGB or BGR
     */
    ColorOrder colorOrder = ColorOrder::BGR;
    /**
     * Are colors interleaved (R1G1B1, R2G2B2, ...) or planar (R1R2..., G1G2..., B1B2)
     */
    bool interleaved = true;
    /**
     * Are values FP16 type (0.0 - 255.0)
     */
    bool fp16 = false;

    /**
     * Preview frame output height
     */
    uint32_t previewHeight = 300;
    /**
     * Preview frame output width
     */
    uint32_t previewWidth = 300;

    /**
     * Preview frame output width
     */
    int32_t videoWidth = AUTO;

    /**
     * Preview frame output height
     */
    int32_t videoHeight = AUTO;

    /**
     * Preview frame output width
     */
    int32_t stillWidth = AUTO;

    /**
     * Preview frame output height
     */
    int32_t stillHeight = AUTO;

    /**
     * Select the camera sensor resolution
     */
    SensorResolution resolution = SensorResolution::THE_1080_P;
    /**
     * Camera sensor FPS
     */
    float fps = 30.0;

    /**
     * Initial sensor crop, -1 signifies center crop
     */
    float sensorCropX = AUTO;
    float sensorCropY = AUTO;

    /**
     * Whether to wait for config at 'inputConfig' io
     */
    bool inputConfigSync = false;

    /**
     * Whether to keep aspect ratio of input (video size) or not
     */
    bool previewKeepAspectRatio = true;

    /**
     * Configure scaling for `isp` output.
     */
    IspScale ispScale;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ColorCameraProperties,
                                   initialControl,
                                   boardSocket,
                                   imageOrientation,
                                   colorOrder,
                                   interleaved,
                                   fp16,
                                   previewHeight,
                                   previewWidth,
                                   videoWidth,
                                   videoHeight,
                                   stillWidth,
                                   stillHeight,
                                   resolution,
                                   fps,
                                   sensorCropX,
                                   sensorCropY,
                                   inputConfigSync,
                                   previewKeepAspectRatio,
                                   ispScale);

}  // namespace dai
