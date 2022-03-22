/**
 * \file utils.h @brief Utilities
 */

// Created by amourao on 26-06-2019.
#pragma once

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>

#include "logger.h"
#include "spdlog/sinks/basic_file_sink.h"
#include <yaml-cpp/yaml.h>

namespace moetsi::ssp {

/**
 * @brief Get current time in ms.
 * \return ms since UTC epoch
 */
uint64_t CurrentTimeMs();

/**
 * @brief Get current time in usec/microseconds.
 * \return usec since UTC epoch
 */
uint64_t CurrentTimeUs();

/**
 * @brief Get current time in ns/nanosecconds
 * \return nsec since UTC epoch
 */
uint64_t CurrentTimeNs();

/**
 * @brief Build a random string
 */
std::string RandomString(size_t length);

/**
 * @brief Setup SSP logging.
 * \param general_parameters configuration
 */
void SetupLogging(YAML::Node &general_parameters);

/**
 * @brief Setup SSP logging.
 * \param level logging level
 * \param file log file
 */
void SetupLogging(std::string &level, std::string &file);

#ifdef _av_err2str
#undef _av_err2str
#endif 

#ifdef _WIN32

#define _av_err2str(errnum)                                                     \
  av_make_error_string((char *)_malloca(AV_ERROR_MAX_STRING_SIZE),             \
                       AV_ERROR_MAX_STRING_SIZE, errnum)
#else
#define _av_err2str(errnum)                                                     \
  av_make_error_string((char *)__builtin_alloca(AV_ERROR_MAX_STRING_SIZE),     \
                       AV_ERROR_MAX_STRING_SIZE, errnum)
#endif

/// *** #error TODO load yaml from string or path ~ LoadFile -> Load

}
