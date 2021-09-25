#pragma once

#include <vector>

#include "depthai-shared/common/CameraBoardSocket.hpp"

namespace dai {

/// StereoRectification structure
struct StereoRectification {
    std::vector<std::vector<float>> rectifiedRotationLeft, rectifiedRotationRight;
    CameraBoardSocket leftCameraSocket = CameraBoardSocket::AUTO, rightCameraSocket = CameraBoardSocket::AUTO;
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(StereoRectification, rectifiedRotationLeft, rectifiedRotationRight, leftCameraSocket, rightCameraSocket);
};

}  // namespace dai
