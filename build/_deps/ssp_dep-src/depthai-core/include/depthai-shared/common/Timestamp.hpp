#pragma once

// std
#include <cstdint>

// libraries
#include <chrono>

#include "nlohmann/json.hpp"

namespace dai {

/// Timestamp structure
struct Timestamp {
    int64_t sec = 0, nsec = 0;
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> get() const {
        using namespace std::chrono;
        return time_point<steady_clock, steady_clock::duration>{seconds(sec) + nanoseconds(nsec)};
    }

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(Timestamp, sec, nsec);
};

}  // namespace dai
