#pragma once

#include <depthai-shared/common/ProcessorType.hpp>
#include <depthai-shared/common/optional.hpp>
#include <nlohmann/json.hpp>

namespace dai {

/**
 * Specify ScriptProperties options such as script uri, script name, ...
 */
struct ScriptProperties {
    /**
     * Uri which points to actual script
     */
    std::string scriptUri = "";

    /**
     * Name of script
     */
    std::string scriptName = "<script>";

    /**
     * Which processor should execute the script
     */
    ProcessorType processor = ProcessorType::LEON_MSS;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ScriptProperties, scriptUri, scriptName, processor);

}  // namespace dai
