//
// Created by amourao on 26-06-2019.
//

#pragma once

#include <vector>
#include <iterator>

#include <cereal/cereal.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/archives/binary.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>

#include <libavcodec/avcodec.h>

#include "Utils.h"

struct FrameStruct {
    unsigned short messageType;
    std::vector<unsigned char> colorFrame;
    std::vector<unsigned char> depthFrame;
    enum AVCodecID colorCodecId;
    enum AVCodecID depthCodecId;

    std::string sceneDesc;
    unsigned int sensorId;
    unsigned int deviceId;
    unsigned int frameId;
    std::vector<unsigned long> timestamp;

    template<class Archive>
    void serialize(Archive &ar) {
        ar(messageType, colorFrame, depthFrame, colorCodecId, depthCodecId, sceneDesc, sensorId, deviceId, frameId,
           timestamp);
    }

    cv::Mat getColorFrame() {
        if (messageType == 0)
            return cv::imdecode(colorFrame, CV_LOAD_IMAGE_UNCHANGED);
        else
            return cv::Mat();
    }

    cv::Mat getDepthFrame() {
        //TODO: check if it is reading into 16 bit cv::Mat
        if (messageType == 0)
            return cv::imdecode(depthFrame, CV_LOAD_IMAGE_UNCHANGED);
        else
            return cv::Mat();
    }

    static FrameStruct parseFrameStruct(std::string &data) {
        FrameStruct frameIn;
        std::istringstream is(data, std::ios::binary);
        {
            cereal::BinaryInputArchive iarchive(is);
            iarchive(frameIn);
        }
        return frameIn;
    }


    static FrameStruct parseFrameStruct(std::vector<unsigned char> &data, size_t dataSize) {
        FrameStruct frameIn;
        std::istringstream is(std::string(data.begin(), data.begin() + dataSize), std::ios::binary);
        {
            cereal::BinaryInputArchive iarchive(is);
            iarchive(frameIn);
        }
        return frameIn;
    }

    static std::string getStructBytes(FrameStruct frame) {
        std::ostringstream os(std::ios::binary);

        {
            cereal::BinaryOutputArchive oarchive(os);
            oarchive(frame);
        }

        return os.str();

    }


};


//TODO: remove if not needed
template<typename T>
const std::vector<uint8_t> serialize(const T &t) {
    std::stringstream ss(std::ios::binary | std::ios::out | std::ios::in);
    cereal::BinaryOutputArchive class_to_ss = {ss};
    class_to_ss(t);

    return {std::istream_iterator<uint8_t>(ss),
            std::istream_iterator<uint8_t>()};
}

template<typename T>
T deserialize(std::vector<uint8_t> &dat) {
    std::stringstream ss(std::ios::binary | std::ios::out | std::ios::in);
    cereal::BinaryOutputArchive arr_to_ss = {ss};
    arr_to_ss(cereal::binary_data(dat.data(), dat.size()));

    cereal::BinaryInputArchive ss_to_MyClass(ss);
    T t;
    ss_to_MyClass(t);
    return t;
}


