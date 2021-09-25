#pragma once

#include <nlohmann/json.hpp>

namespace dai {

/**
 * CpuUsage structure
 *
 * Average usage in percent and time span of the average (since last query)
 */
struct CpuUsage {
    /**
     *  Average CPU usage, expressed with a normalized value (0-1)
     */
    float average;
    /**
     *  Time span in which the average was calculated in milliseconds
     */
    int32_t msTime;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CpuUsage, average, msTime);

}  // namespace dai