#pragma once

// std
#include <cstdint>

namespace dai
{
namespace bootloader
{

enum class Section : std::int32_t {
    HEADER, BOOTLOADER, BOOTLOADER_CONFIG, APPLICATION
};

} // namespace bootloader
} // namespace dai

