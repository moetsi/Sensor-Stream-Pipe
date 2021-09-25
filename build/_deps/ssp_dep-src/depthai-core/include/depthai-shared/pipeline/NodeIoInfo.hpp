#pragma once

#include <nlohmann/json.hpp>

namespace dai {

/// NodeIo informations such as name, type, ...
struct NodeIoInfo {
    enum class Type { MSender, SSender, MReceiver, SReceiver };

    std::string name;
    Type type = Type::SReceiver;
    bool blocking = true;
    int queueSize = 8;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(NodeIoInfo, name, type, blocking, queueSize);

}  // namespace dai
