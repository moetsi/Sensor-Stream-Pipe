#pragma once

// std
#include <cstdint>

namespace dai
{
namespace bootloader
{

enum class Memory : std::int32_t {
    FLASH, EMMC
};

} // namespace bootloader
} // namespace dai
