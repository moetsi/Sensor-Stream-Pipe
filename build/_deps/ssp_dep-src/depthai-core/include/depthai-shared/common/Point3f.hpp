#pragma once

// std
#include <cstdint>

// libraries
#include "nlohmann/json.hpp"

namespace dai {

/**
 * Point3f structure
 *
 * x,y,z coordinates that define a 3D point.
 */
struct Point3f {
    Point3f() {}
    Point3f(float x, float y, float z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }
    float x = 0, y = 0, z = 0;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(Point3f, x, y, z);
};

}  // namespace dai
