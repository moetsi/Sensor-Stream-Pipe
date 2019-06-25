#pragma once

#include <vector>

struct FrameStruct {
    unsigned short messageType;
    std::vector<unsigned char> colorFrame;
    std::vector<unsigned char> depthFrame;
    unsigned int sensorId;
    unsigned int deviceId;
    unsigned int frameId;
    std::vector<unsigned long> timestamp;
};
