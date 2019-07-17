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

//TODO: merge with video frame struct and make it more generic
struct FrameStruct {
    unsigned short messageType;
    std::vector<unsigned char> colorFrame;
    std::vector<unsigned char> depthFrame;

    std::string sceneDesc;
    unsigned int sensorId;
    unsigned int deviceId;
    unsigned int frameId;
    std::vector<unsigned long> timestamp;

    template<class Archive>
    void serialize(Archive &ar) {
        ar(messageType, colorFrame, depthFrame, sceneDesc, sensorId, deviceId, frameId,
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

struct CodecParamsStruct {
    std::vector<unsigned char> data;
    std::vector<unsigned char> extra_data;

    CodecParamsStruct() {}

    CodecParamsStruct(std::vector<unsigned char> d, std::vector<unsigned char> ed) : data(d), extra_data(ed) {}


    void setData(std::vector<unsigned char> &d) {
        data = d;
    }

    void setExtraData(std::vector<unsigned char> &ed) {
        extra_data = ed;
    }

    AVCodecParameters *getParams() {
        AVCodecParameters *results = avcodec_parameters_alloc();
        results->extradata = NULL;
        results->extradata_size = 0;
        memcpy(results, &data[0], data.size());
        results->extradata = (uint8_t *) av_mallocz(extra_data.size() + AV_INPUT_BUFFER_PADDING_SIZE);
        memcpy(results->extradata, &extra_data[0], extra_data.size());
        results->extradata_size = extra_data.size();
        return results;
    }

    template<class Archive>
    void serialize(Archive &ar) {
        ar(data, extra_data);
    }
};

//TODO: merge with frame struct and make it more generic
struct VideoFrameStruct {
    unsigned short messageType;

    std::string streamId;

    std::vector<std::vector<unsigned char>> frames;
    std::vector<CodecParamsStruct> codec_data;

    std::string sceneDesc;
    unsigned int sensorId;
    unsigned int deviceId;
    unsigned int frameId;
    std::vector<unsigned long> timestamps;



    template<class Archive>
    void serialize(Archive &ar) {
        ar(messageType, streamId, frames, codec_data, sceneDesc, sensorId, deviceId, frameId,
           timestamps);
    }

    cv::Mat getColorFrame() {
        return cv::Mat();
    }

    cv::Mat getDepthFrame() {
        return cv::Mat();
    }

    static VideoFrameStruct parseFrameStruct(std::string &data) {
        VideoFrameStruct frameIn;
        std::istringstream is(data, std::ios::binary);
        {
            cereal::BinaryInputArchive iarchive(is);
            iarchive(frameIn);
        }
        return frameIn;
    }


    static VideoFrameStruct parseFrameStruct(std::vector<unsigned char> &data, size_t dataSize) {
        VideoFrameStruct frameIn;
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

template<typename T>
const std::string serialize_to_str(const T &t) {
    std::ostringstream os(std::ios::binary);
    {
        cereal::BinaryOutputArchive oarchive(os);
        oarchive(t);
    }

    return os.str();
}

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


