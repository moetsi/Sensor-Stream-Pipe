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

#include "../utils/utils.h"

struct CameraCalibrationStruct {
  // 0: Kinect parameters
  short type = -1;
  std::vector<unsigned char> data;
  std::vector<unsigned char> extra_data;

  CameraCalibrationStruct() {}

  CameraCalibrationStruct(unsigned int t, std::vector<unsigned char> d,
                          std::vector<unsigned char> ed)
      : type(t), data(d), extra_data(ed) {}

  template <class Archive> void serialize(Archive &ar) {
    ar(type, data, extra_data);
  }
};

struct CodecParamsStruct {
  // 0: av parameters, 1: nvPipe parameters, 2: zDepth parameters
  short type = -1;
  std::vector<unsigned char> data;
  std::vector<unsigned char> extra_data;

  CodecParamsStruct() {}

  CodecParamsStruct(unsigned int t, std::vector<unsigned char> d,
                    std::vector<unsigned char> ed)
      : type(t), data(d), extra_data(ed) {}

  void SetData(std::vector<unsigned char> &d) { data = d; }

  void SetExtraData(std::vector<unsigned char> &ed) { extra_data = ed; }

  template <class Archive> void serialize(Archive &ar) {
    ar(type, data, extra_data);
  }
};

struct FrameStruct {

  // message id, currenly set to 0
  //This is to be used as "versioning", so if how messages are updated so that Sensor Stream Client
  //must interpret different "versions" of messages then this field will indicate the message version
  unsigned short message_type;

  // 0 for color, 1 for depth, 2 for ir, 3 for confidence, 4 for object
  unsigned short frame_type;

  // 0 for image frames, 1 for libav packets, 2 for raw RGBA data, 3 for raw
  // GRAY16LE data, 4 for NvPipe packets, 5 for raw 32FC1 data, 6 for YUV data
  // 7 for raw U8C1 data //8 for object-human data
  //This is used to select the decoder on the "receiving" side of the Pipe
  //Not all frame_type + frame_data_type combinations "make sense" or will be used
  unsigned short frame_data_type;

  // random 16 char string that uniquely ids the frame stream
  //Some decoders (like video) are stateful and so must keep track of streams
  //This is automatically generated
  std::string stream_id;

  // frame binary data
  //We use a vector to know the size, basically a vector of bytes to store binary data
  std::vector<unsigned char> frame;

  // codec info for video frames, null for image frames
  //Video decoder needs to know about the last receive frame
  //Requires to know the codec as well as additional parameters
  CodecParamsStruct codec_data;

  // codec info for video frames, null for image frames
  CameraCalibrationStruct camera_calibration_data;

  // optional: scene description
  std::string scene_desc;

  // 0 for color, 1 for depth: currently redundant with frameType, but
  // distinction may be needed in the future
  unsigned int sensor_id;

  // integer device id: distingish between devices in the same scene
  //Can be set by user
  unsigned int device_id;

  // current frame number (increases over time)
  //Increases by 1 for each frame automatically when SSP server starts
  unsigned int frame_id;

  //Use for logging and timing to understand processing speeds
  std::vector<unsigned long> timestamps;

  //Serialize method (not used by Server but is available)
  template <class Archive> void serialize(Archive &ar) {
    ar(message_type, frame_type, frame_data_type, stream_id, frame, codec_data,
       camera_calibration_data, scene_desc, sensor_id, device_id, frame_id,
       timestamps);
  }
};

template <typename T>
static const std::string CerealStructToString(const T &t) {
  std::ostringstream os(std::ios::binary);
  {
    cereal::BinaryOutputArchive oarchive(os);
    oarchive(t);
  }

  return os.str();
}

template <typename T> static const std::string FrameStructToString(const T *t) {
  std::ostringstream os(std::ios::binary);
  {
    cereal::BinaryOutputArchive oarchive(os);
    oarchive(*t);
  }

  return os.str();
}

template <typename T> static T ParseCerealStructFromString(std::string &data) {
  T frame_in;
  std::istringstream is(data, std::ios::binary);
  {
    cereal::BinaryInputArchive iarchive(is);
    iarchive(frame_in);
  }
  return frame_in;
}
