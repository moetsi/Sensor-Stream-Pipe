#pragma once

#include <depthai-shared/datatype/RawCameraControl.hpp>
#include <nlohmann/json.hpp>

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/common/CameraImageOrientation.hpp"

namespace dai {

/**
 * Specify properties for MonoCamera such as camera ID, ...
 */
struct MonoCameraProperties {
    /**
     * Select the camera sensor resolution: 1280×720, 1280×800, 640×400
     */
    enum class SensorResolution : int32_t { THE_720_P, THE_800_P, THE_400_P };

    /*
     * Initial controls applied to MonoCamera node
     */
    RawCameraControl initialControl;

    /**
     * Which socket will mono camera use
     */
    CameraBoardSocket boardSocket = CameraBoardSocket::AUTO;

    /**
     * Camera sensor image orientation / pixel readout
     */
    CameraImageOrientation imageOrientation = CameraImageOrientation::AUTO;

    /**
     * Select the camera sensor resolution
     */
    SensorResolution resolution = SensorResolution::THE_720_P;
    /**
     * Camera sensor FPS
     */
    float fps = 30.0;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MonoCameraProperties, initialControl, boardSocket, imageOrientation, resolution, fps);

}  // namespace dai
