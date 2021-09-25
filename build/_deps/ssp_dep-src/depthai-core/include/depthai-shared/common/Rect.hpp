#pragma once

// libraries
#include <algorithm>

#include "depthai-shared/common/Point2f.hpp"
#include "depthai-shared/common/Size2f.hpp"
#include "nlohmann/json.hpp"

namespace dai {

/**
 * Rect structure
 *
 * x,y coordinates together with width and height that define a rectangle.
 * Can be either normalized [0,1] or absolute representation.
 */
struct Rect {
    // default constructor
    Rect() : x(0), y(0), width(0), height(0) {}
    Rect(float x, float y, float width, float height) {
        this->x = x;
        this->y = y;
        this->width = width;
        this->height = height;
    }

    Rect(const Rect& r) : x(r.x), y(r.y), width(r.width), height(r.height) {}
    Rect(const Point2f& org, const Size2f& sz) : x(org.x), y(org.y), width(sz.width), height(sz.height) {}
    Rect(const Point2f& pt1, const Point2f& pt2) {
        x = std::min(pt1.x, pt2.x);
        y = std::min(pt1.y, pt2.y);
        width = std::max(pt1.x, pt2.x) - x;
        height = std::max(pt1.y, pt2.y) - y;
    }

    Rect& operator=(const Rect& r) {
        x = r.x;
        y = r.y;
        width = r.width;
        height = r.height;
        return *this;
    }

    /**
     * The top-left corner.
     */
    Point2f topLeft() const {
        return Point2f(x, y);
    }

    /**
     * The bottom-right corner
     */
    Point2f bottomRight() const {
        return Point2f(x + width, y + height);
    }

    /**
     * Size (width, height) of the rectangle
     */
    Size2f size() const {
        return Size2f(width, height);
    }

    /**
     * Area (width*height) of the rectangle
     */
    float area() const {
        return width * height;
    }

    /**
     * True if rectangle is empty.
     */
    bool empty() const {
        return width <= 0 || height <= 0;
    }

    /**
     * Checks whether the rectangle contains the point.
     */
    bool contains(const Point2f& pt) const {
        return x <= pt.x && pt.x < x + width && y <= pt.y && pt.y < y + height;
    }

    /**
     * Whether rectangle is normalized (coordinates in [0,1] range) or not.
     */
    bool isNormalized() const {
        if(x + width <= 1.f && y + height <= 1.f) return true;
        return !(x == static_cast<int>(x) && y == static_cast<int>(y) && width == static_cast<int>(width) && height == static_cast<int>(height));
    }

    /**
     * Denormalize rectangle.
     * @param width Destination frame width.
     * @param height Destination frame height.
     */
    Rect denormalize(int width, int height) {
        if(isNormalized()) {
            float _x = std::round(this->x * width);
            float _y = std::round(this->y * height);
            float _width = std::round(this->width * width);
            float _height = std::round(this->height * height);
            return Rect(_x, _y, _width, _height);
        }
        return *this;
    }

    /**
     * Normalize rectangle.
     * @param width Source frame width.
     * @param height Source frame height.
     */
    Rect normalize(int width, int height) {
        if(isNormalized()) {
            return *this;
        }
        float _x = this->x / width;
        float _y = this->y / height;
        float _width = this->width / width;
        float _height = this->height / height;
        return Rect(_x, _y, _width, _height);
    }

    float x;       // x coordinate of the top-left corner
    float y;       // y coordinate of the top-left corner
    float width;   // width of the rectangle
    float height;  // height of the rectangle
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Rect, x, y, width, height);

}  // namespace dai