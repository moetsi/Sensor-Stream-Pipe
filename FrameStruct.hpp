#pragma once

#include <vector>

#include <cereal/cereal.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/archives/binary.hpp>

struct FrameStruct {
    unsigned short messageType;
    std::vector<unsigned char> colorFrame;
    std::vector<unsigned char> depthFrame;
    unsigned int sensorId;
    unsigned int deviceId;
    unsigned int frameId;
    std::vector<unsigned long> timestamp;

    template <class Archive>
    void serialize( Archive & ar )
    {
        ar( messageType, colorFrame, depthFrame, sensorId, deviceId, frameId, timestamp);
    }
};
