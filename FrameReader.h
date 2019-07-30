//
// Created by amourao on 27-06-2019.
//

#pragma once

#include <iostream>
#include <fstream>
#include <vector>

#include <cereal/archives/binary.hpp>

#include "FrameStruct.hpp"

class FrameReader {
private:

    unsigned int currentFrameCounter;
    unsigned int fps;
    std::string sceneDesc;
    unsigned int sensorId;
    unsigned int deviceId;
    unsigned int frameType;
    std::string streamId;

    FrameStruct currentFrameInternal;

    std::vector<unsigned char> readFile(std::string &filename);

    FrameStruct createFrameStruct(unsigned int frameId);

    std::string getStructBytes(FrameStruct frame);

public:
    std::vector<std::string> frameLines;

    FrameReader(std::string filename);

    void reset();

    void goToFrame(unsigned int frameId);

    bool hasNextFrame();

    void nextFrame();

    FrameStruct currentFrame();

    std::string currentFramePath(int i);

    std::string currentFrameBytes();

    unsigned int currentFrameId();

    std::string getSceneDesc();

    unsigned int getFps();

    unsigned int getSensorId();

    unsigned int getDeviceId();

    unsigned int getFrameType();

    std::string getStreamId();


};


