#pragma once

// std
#include <cstdint>

// libraries
#include "nlohmann/json.hpp"

// shared
#include "depthai-shared/common/Point2f.hpp"
#include "depthai-shared/common/Size2f.hpp"

namespace dai {

/// RotatedRect structure
struct RotatedRect {
    Point2f center;
    Size2f size;
    /// degrees, increasing clockwise
    float angle;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(RotatedRect, center, size, angle);
};

}  // namespace dai
