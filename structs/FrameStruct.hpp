//
// Created by amourao on 26-06-2019.
//

#pragma once

#include <iterator>
#include <vector>

#include <cereal/archives/binary.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/vector.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
}

#include "../utils/Utils.h"

struct CodecParamsStruct {
  // 0: av parameters, 1: nvPipe parameters, 2: zDepth parameters
  uint type;
  std::vector<unsigned char> data;
  std::vector<unsigned char> extra_data;

  CodecParamsStruct() {}

  CodecParamsStruct(uint t, std::vector<unsigned char> d, std::vector<unsigned char> ed)
          : type(t), data(d), extra_data(ed) {}

  void setData(std::vector<unsigned char> &d) { data = d; }

  void setExtraData(std::vector<unsigned char> &ed) { extra_data = ed; }

  // TODO: pull this function  into other file, to avoid unneeded AVCodec
  // dependecy
  AVCodecParameters *getParams() {
    if (type != 0)
      return NULL;
    AVCodecParameters *results = avcodec_parameters_alloc();
    results->extradata = NULL;
    results->extradata_size = 0;
    memcpy(results, &data[0], data.size());
    results->extradata =
            (uint8_t *) av_mallocz(extra_data.size() + AV_INPUT_BUFFER_PADDING_SIZE);
    memcpy(results->extradata, &extra_data[0], extra_data.size());
    results->extradata_size = extra_data.size();
    return results;
  }

  template<class Archive>
  void serialize(Archive &ar) { ar(type, data, extra_data); }
};

struct FrameStruct {

  // message id, currenly set to 0
  unsigned short messageType;

  // 0 for color, 1 for depth
  unsigned short frameType;

  // 0 for image frames, 1 for libav packets, 2 for raw RGBA data, 3 for raw
  // GRAY16LE data, 4 for NvPipe packets
  unsigned short frameDataType;

  // random 16 char string that uniquely ids the frame stream
  std::string streamId;

  // frame binary data
  std::vector<unsigned char> frame;

  // codec info for video frames, null for image frames
  CodecParamsStruct codec_data;

  // optional: scene description
  std::string sceneDesc;

  // 0 for color, 1 for depth: currently redundant with frameType, but
  // distinction may be needed in the future
  unsigned int sensorId;

  // integer device id: distingish between devices in the same scene
  unsigned int deviceId;

  // current frame number (increases over time)
  unsigned int frameId;

  std::vector<unsigned long> timestamps;

  template<class Archive>
  void serialize(Archive &ar) {
    ar(messageType, frameType, frameDataType, streamId, frame, codec_data, sceneDesc, sensorId,
       deviceId, frameId, timestamps);
  }
};

template<typename T>
static const std::string cerealStructToString(const T &t) {
  std::ostringstream os(std::ios::binary);
  {
    cereal::BinaryOutputArchive oarchive(os);
    oarchive(t);
  }

  return os.str();
}

template<typename T>
static const std::string frameStructToString(const T *t) {
  std::ostringstream os(std::ios::binary);
  {
    cereal::BinaryOutputArchive oarchive(os);
    oarchive(*t);
  }

  return os.str();
}


template<typename T>
static T parseCerealStructFromString(std::string &data) {
  T frameIn;
  std::istringstream is(data, std::ios::binary);
  {
    cereal::BinaryInputArchive iarchive(is);
    iarchive(frameIn);
  }
  return frameIn;
}
