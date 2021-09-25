#pragma once

#include <depthai-shared/common/optional.hpp>
#include <nlohmann/json.hpp>

#include "depthai-shared/common/ProcessorType.hpp"

namespace dai {

/**
 * Specify message and processor placement of MyProducer node
 */
struct MyProducerProperties {
    /**
     * Message to be sent forward
     */
    tl::optional<std::string> message;

    /**
     * On which processor the node will be placed
     */
    ProcessorType processorPlacement = ProcessorType::LEON_CSS;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MyProducerProperties, message, processorPlacement);

}  // namespace dai
