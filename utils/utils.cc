/**
 * \file Utils.cc @brief Utilities
 */
// Created by amourao on 26-06-2019.
#include "utils.h"

#include <random>
#include <chrono>

namespace moetsi::ssp {

namespace {
  thread_local std::mt19937_64 rng(std::chrono::steady_clock::now().time_since_epoch().count());
}

uint64_t CurrentTimeMs() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

uint64_t CurrentTimeUs() {
  using namespace std::chrono;
  return duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
}

uint64_t CurrentTimeNs() {
  using namespace std::chrono;
  return duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
}

std::string RandomString(size_t length) {
  auto randchar = []() -> char {
    const char charset[] = "0123456789"
                           "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                           "abcdefghijklmnopqrstuvwxyz";
    const size_t max_index = (sizeof(charset) - 1);
    return charset[
      // rand() 
      rng() % max_index];
  };
  std::string str(length, 0);
  std::generate_n(str.begin(), length, randchar);
  return str;
}

void SetupLogging(YAML::Node &general_parameters) {
  if (general_parameters["log_level"].IsDefined())
    spdlog::set_level(spdlog::level::from_str(
        general_parameters["log_level"].as<std::string>()));
  if (general_parameters["log_file"].IsDefined())
    spdlog::set_default_logger(spdlog::basic_logger_mt(
        "basic_logger", general_parameters["log_file"].as<std::string>()));
}

void SetupLogging(const std::string &level, const std::string &file) {
  spdlog::set_level(spdlog::level::from_str(level));
  if (!file.empty())
    spdlog::set_default_logger(spdlog::basic_logger_mt("basic_logger", file));
}

} // namespace moetsi::ssp
