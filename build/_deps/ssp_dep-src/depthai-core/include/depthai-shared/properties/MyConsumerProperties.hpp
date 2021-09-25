#pragma once

#include <nlohmann/json.hpp>

#include "depthai-shared/common/ProcessorType.hpp"

namespace dai {

/**
 * Specify message and processor placement of MyConsumer node
 */
struct MyConsumerProperties {
    /**
     * On which processor the node will be placed
     */
    ProcessorType processorPlacement;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MyConsumerProperties, processorPlacement);

}  // namespace dai
