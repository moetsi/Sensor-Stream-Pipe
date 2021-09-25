#pragma once

// std
#include <cstdint>

namespace dai
{
namespace bootloader
{

enum class Type : std::int32_t {
    USB, NETWORK
};

} // namespace bootloader
} // namespace dai
