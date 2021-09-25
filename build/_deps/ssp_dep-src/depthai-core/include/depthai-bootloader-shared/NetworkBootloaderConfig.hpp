#pragma once

// std
#include <cstdint>
#include <chrono>

// project
#include "Structure.hpp"

namespace dai
{
namespace bootloader
{

// Config
/* TODO(themarpe)
struct NetworkBootloaderConfig {
    char ip[16];
    char subnetMask[16];
    char gateway[16];
};
*/


// Structure
struct NetworkBootloaderStructure : Structure {

    constexpr static long HEADER_OFFSET = 0;
    constexpr static long HEADER_SIZE = 512;
    constexpr static long CONFIG_SIZE = 16 * 1024;
    constexpr static long BOOTLOADER_OFFSET = HEADER_OFFSET + HEADER_SIZE;
    constexpr static long BOOTLOADER_SIZE = 8 * 1024 * 1024 - CONFIG_SIZE;
    constexpr static long CONFIG_OFFSET = BOOTLOADER_OFFSET + BOOTLOADER_SIZE;
    constexpr static long APPLICATION_OFFSET = CONFIG_OFFSET + CONFIG_SIZE;

    NetworkBootloaderStructure() : Structure({
        {Section::HEADER, HEADER_OFFSET},
        {Section::BOOTLOADER_CONFIG, CONFIG_OFFSET},
        {Section::BOOTLOADER, BOOTLOADER_OFFSET},
        {Section::APPLICATION, APPLICATION_OFFSET},
    }, {
        {Section::HEADER, HEADER_SIZE},
        {Section::BOOTLOADER_CONFIG, CONFIG_SIZE},
        {Section::BOOTLOADER, BOOTLOADER_SIZE},
        {Section::APPLICATION, 0},
    }) {}

};

static const NetworkBootloaderStructure networkBootloaderStructure;

} // namespace bootloader
} // namespace dai


