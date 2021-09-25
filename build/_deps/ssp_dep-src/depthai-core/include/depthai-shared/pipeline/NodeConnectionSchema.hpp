#pragma once

#include <nlohmann/json.hpp>

namespace dai {

/**
 * Specifies a connection between nodes IOs
 */
struct NodeConnectionSchema {
    int64_t node1Id = -1;
    std::string node1Output;
    int64_t node2Id = -1;
    std::string node2Input;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(NodeConnectionSchema, node1Id, node1Output, node2Id, node2Input);

}  // namespace dai
