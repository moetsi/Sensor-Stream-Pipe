//
// Created by amourao on 27-06-2019.
//

#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <atomic>


#include <k4a/k4a.h>

#include <cereal/archives/binary.hpp>

#include "../structs/FrameStruct.hpp"
#include "../utils/VideoUtils.h"

#include <k4a/k4a.h>
#include <k4arecord/record.h>

extern std::atomic_bool exiting;


inline static uint32_t k4a_convert_fps_to_uint(k4a_fps_t fps) {
    uint32_t fps_int;
    switch (fps) {
        case K4A_FRAMES_PER_SECOND_5:
            fps_int = 5;
            break;
        case K4A_FRAMES_PER_SECOND_15:
            fps_int = 15;
            break;
        case K4A_FRAMES_PER_SECOND_30:
            fps_int = 30;
            break;
        default:
            fps_int = 0;
            break;
    }
    return fps_int;
}


// call k4a_device_close on every failed CHECK
#define CHECK(x, device)                                                                                               \
    {                                                                                                                  \
        auto retval = (x);                                                                                             \
        if (retval)                                                                                                    \
        {                                                                                                              \
            std::cerr << "Runtime error: " << #x << " returned " << retval << std::endl;                               \
            k4a_device_close(device);                                                                                  \
            exit(1);                                                                                                  \
        }                                                                                                              \
    }

class KinectReader {
private:

    std::vector<uint> currentFrameCounter;
    unsigned int fps;
    unsigned int sensorId;
    unsigned int deviceId;
    std::string streamId;

    uint8_t device_index;
    k4a_device_configuration_t *device_config;
    bool record_imu;
    int32_t absoluteExposureValue;
    k4a_device_t device;
    k4a_record_t recording;
    k4a_capture_t capture;
    k4a_wait_result_t result = K4A_WAIT_RESULT_TIMEOUT;
    int32_t timeout_ms;
    clock_t recording_start;

    FrameStruct frameTemplate;

    std::vector<FrameStruct> currFrame;


public:

    KinectReader(uint8_t _device_index, k4a_device_configuration_t *_device_config, int32_t _absoluteExposureValue);

    ~KinectReader();

    void reset();

    bool hasNextFrame();

    void nextFrame();

    std::vector<FrameStruct> currentFrame();

    FrameStruct currentFrame(uint type);

    uint getFps();

    std::vector<uint> getType();

};


