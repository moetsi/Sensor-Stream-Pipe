#pragma once

#include <nlohmann/json.hpp>

namespace dai {

/**
 * MemoryInfo structure
 *
 * Free, remaining and total memory stats
 */
struct MemoryInfo {
    int64_t remaining;
    int64_t used;
    int64_t total;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MemoryInfo, remaining, used, total);

}  // namespace dai