#pragma once

#include <nlohmann/json.hpp>

namespace dai {

/**
 * Specify properties for SPIOut node
 */
struct SPIOutProperties {
    /**
     * Name of stream
     */
    std::string streamName;

    /**
     * SPI bus to use
     */
    int busId = 0;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SPIOutProperties, streamName, busId);

}  // namespace dai
