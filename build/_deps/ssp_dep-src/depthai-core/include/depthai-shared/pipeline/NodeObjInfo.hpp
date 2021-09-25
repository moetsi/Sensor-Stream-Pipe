#pragma once

#include <nlohmann/json.hpp>
#include <unordered_map>

#include "NodeIoInfo.hpp"
namespace dai {

/// NodeObj information structure
struct NodeObjInfo {
    int64_t id = -1;
    std::string name;
    nlohmann::json properties;
    std::unordered_map<std::string, NodeIoInfo> ioInfo;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(NodeObjInfo, id, name, properties, ioInfo);

}  // namespace dai
