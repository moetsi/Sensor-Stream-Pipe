#pragma once

// std
#include <map>

// project
#include "Section.hpp"

namespace dai
{
namespace bootloader
{

// Memory section structure
struct Structure {
    Structure() = default;
    std::map<Section, long> offset, size;
protected:
    Structure(decltype(offset) a, decltype(size) b) : offset(a), size(b) {}
};

} // namespace bootloader
} // namespace dai

