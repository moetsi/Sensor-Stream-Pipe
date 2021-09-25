#pragma once

// std
#include <cstdint>

// libraries
#include "nlohmann/json.hpp"

namespace dai {

/**
 * Size2f structure
 *
 * width, height values define the size of the shape/frame
 */
struct Size2f {
    Size2f() {}
    Size2f(float width, float height) {
        this->width = width;
        this->height = height;
    }
    float width = 0, height = 0;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(Size2f, width, height);
};

}  // namespace dai
