//
// Created by amourao on 26-06-2019.
//

#include "FrameReader.h"

FrameReader::FrameReader(std::string filename) {
    // bundle_fusion_apt0;0;0;30

    // std::string sceneDesc;
    // unsigned int sensorId;
    // unsigned int deviceId;

    std::ifstream file(filename);
    std::string line;
    getline(file, line);
    std::string value;

    std::stringstream ss(line);
    getline(ss, sceneDesc, ';');

    std::string sensorIdStr, deviceIdStr, frameCountStr, fpsStr, frameTypeStr;
    getline(ss, deviceIdStr, ';');
    getline(ss, sensorIdStr, ';');
    getline(ss, frameTypeStr, ';');
    getline(ss, fpsStr);

    sensorId = std::stoul(sensorIdStr);
    deviceId = std::stoul(deviceIdStr);
    frameType = std::stoul(frameTypeStr);
    fps = std::stoul(fpsStr);

    // get frame count
    getline(file, frameCountStr);
    unsigned int frameCount = std::stoul(frameCountStr);

    while (getline(file, line))
        frameLines.push_back(line);

    if (frameCount != frameLines.size())
        std::cerr << "Warning: lines read do not match expected size: " << frameLines.size() << " read vs. "
                  << frameCount << " expected." << std::endl;

    streamId = randomString(16);
    reset();
}

std::vector<unsigned char> FrameReader::readFile(std::string &filename) {
    std::streampos fileSize;
    std::ifstream file(filename, std::ios::binary);

    file.seekg(0, std::ios::end);
    fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<unsigned char> fileData(fileSize);
    file.read((char *) &fileData[0], fileSize);
    return fileData;
}

FrameStruct FrameReader::createFrameStruct(unsigned int frameId) {
    // 0;/home/amourao/data/bundle_fusion/apt0/frame-000000.color.jpg;/home/amourao/data/bundle_fusion/apt0/frame-000000.color.jpg

    std::string line = frameLines[frameId];
    std::stringstream ss(line);

    std::string frameIdStr, framePath;

    getline(ss, frameIdStr, ';');
    getline(ss, framePath);

    unsigned int readFrameId = std::stoul(frameIdStr);

    if (readFrameId != currentFrameCounter)
        std::cerr << "Warning: frame ids do not match: " << readFrameId << " read vs. " << currentFrameCounter
                  << " expected." << std::endl;


    std::vector<unsigned char> fileData = readFile(framePath);
    FrameStruct frame = FrameStruct();

    frame.messageType = 0;

    frame.sceneDesc = sceneDesc;
    frame.deviceId = deviceId;
    frame.sensorId = sensorId;
    frame.frameType = frameType;

    frame.frameId = readFrameId;

    frame.frame = fileData;
    frame.streamId = streamId;

    return frame;
}

unsigned int FrameReader::currentFrameId() {
    return currentFrameCounter;
}

std::vector<FrameStruct> FrameReader::currentFrame() {
    std::vector<FrameStruct> v;
    v.push_back(currentFrameInternal);
    return v;
}

void FrameReader::nextFrame() {
    currentFrameCounter += 1;
    currentFrameInternal = createFrameStruct(currentFrameCounter);
}

bool FrameReader::hasNextFrame() {
    return currentFrameCounter + 1 < frameLines.size();
}

void FrameReader::goToFrame(unsigned int frameId) {
    currentFrameCounter = frameId;
    currentFrameInternal = createFrameStruct(currentFrameCounter);
}

void FrameReader::reset() {
    currentFrameCounter = 0;
    currentFrameInternal = createFrameStruct(currentFrameCounter);
}

unsigned int FrameReader::getFps() {
    return fps;
}
