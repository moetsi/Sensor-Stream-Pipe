//  To parse this JSON data, first install
//
//      Boost     http://www.boost.org
//      json.hpp  https://github.com/nlohmann/json
//
//  Then include this file, and then do
//
//     PipelineSchema.hpp data = nlohmann::json::parse(jsonString);

#pragma once

#include <nlohmann/json.hpp>

#include "NodeConnectionSchema.hpp"
#include "NodeObjInfo.hpp"
#include "depthai-shared/properties/GlobalProperties.hpp"

namespace dai {

/**
 * Specifies whole pipeline, nodes, properties and connections between nodes IOs
 */
struct PipelineSchema {
    std::vector<NodeConnectionSchema> connections;
    GlobalProperties globalProperties;
    std::unordered_map<int64_t, NodeObjInfo> nodes;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PipelineSchema, connections, globalProperties, nodes);

}  // namespace dai
