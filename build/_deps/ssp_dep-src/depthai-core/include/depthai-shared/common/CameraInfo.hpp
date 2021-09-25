#pragma once

#include "depthai-shared/common/CameraModel.hpp"
#include "depthai-shared/common/Extrinsics.hpp"

namespace dai {

/// CameraInfo structure
struct CameraInfo {
    uint16_t width = 0, height = 0;
    uint8_t lensPosition = 0;
    std::vector<std::vector<float>> intrinsicMatrix;
    std::vector<float> distortionCoeff;
    Extrinsics extrinsics;
    float specHfovDeg = 0.0f;  // fov in deg
    CameraModel cameraType = CameraModel::Perspective;
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(CameraInfo, cameraType, width, height, specHfovDeg, lensPosition, intrinsicMatrix, distortionCoeff, extrinsics);
};

}  // namespace dai