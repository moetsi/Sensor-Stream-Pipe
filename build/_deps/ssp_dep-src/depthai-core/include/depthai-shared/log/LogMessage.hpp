#pragma once

// std
#include <cstdint>

// libraries
#include <nlohmann/json.hpp>

// project
#include "LogLevel.hpp"
#include "depthai-shared/common/Timestamp.hpp"

namespace dai {

struct LogMessage {
    std::string nodeIdName;
    LogLevel level;
    Timestamp time;
    size_t colorRangeStart{0};
    size_t colorRangeEnd{0};
    std::string payload;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(LogMessage, nodeIdName, level, time, colorRangeStart, colorRangeEnd, payload);

}  // namespace dai
