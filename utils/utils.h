//
// Created by amourao on 26-06-2019.
//

#pragma once

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>

#include "logger.h"
#include "spdlog/sinks/basic_file_sink.h"
#include <yaml-cpp/yaml.h>

uint64_t CurrentTimeMs();

std::string RandomString(size_t length);

void SetupLogging(YAML::Node &general_parameters);

void SetupLogging(std::string &level, std::string &file);

#undef av_err2str

#ifdef _WIN32
#define av_err2str(errnum)                                                     \
  av_make_error_string((char *)_malloca(AV_ERROR_MAX_STRING_SIZE),             \
                       AV_ERROR_MAX_STRING_SIZE, errnum)
#else
#define av_err2str(errnum)                                                     \
  av_make_error_string((char *)__builtin_alloca(AV_ERROR_MAX_STRING_SIZE),     \
                       AV_ERROR_MAX_STRING_SIZE, errnum)
#endif

