//
// Created by amourao on 26-06-2019.
//

#pragma once

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include <yaml-cpp/yaml.h>


uint64_t currentTimeMs();

std::vector<std::string> split(const std::string &str,
                               const std::string &delim);

std::string randomString(size_t length);

void setupLogging(YAML::Node& general_parameters);

void setupLogging(std::string& level, std::string& file);

#undef av_err2str
#define av_err2str(errnum)                                                     \
  av_make_error_string((char *)__builtin_alloca(AV_ERROR_MAX_STRING_SIZE),     \
                       AV_ERROR_MAX_STRING_SIZE, errnum)
