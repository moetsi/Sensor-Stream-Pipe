//
// Created by amourao on 26-06-2019.
//

#pragma once

#include <chrono>

uint64_t current_time_ms() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}
