#pragma once

#include <chrono>
#include <cstdint>

namespace dai {
// channel names
constexpr static const char* XLINK_CHANNEL_PIPELINE_CONFIG = "__pipeline_config";
constexpr static const char* XLINK_CHANNEL_MAIN_RPC = "__rpc_main";
constexpr static const char* XLINK_CHANNEL_TIMESYNC = "__timesync";
constexpr static const char* XLINK_CHANNEL_LOG = "__log";

// usb buffer maximum size
constexpr static std::uint32_t XLINK_USB_BUFFER_MAX_SIZE = 5 * 1024 * 1024;

// Watchdog timeout
constexpr static const std::chrono::milliseconds XLINK_WATCHDOG_TIMEOUT{1500};

}  // namespace dai
