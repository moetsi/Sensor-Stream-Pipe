/**
 * \file frame_struct.h @brief Frame struct definition. "Universal" frame data type.
 */
// Created by amourao on 26-06-2019.
#pragma once
#include <vector>

#ifndef __MOETSI_RAAS__
#include <iterator>
#include <cereal/archives/binary.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/vector.hpp>
#include "../utils/utils.h"
#endif //!__MOETSI_RAAS__

namespace moetsi::ssp {

/**
 * @brief Camera calibration type i.e. the kind of sensor calibration data present.
 */ 
enum class CameraCalibrationType: short {
    /** Default camera calibration type.  */
    CameraCalibrationTypeDefault = -1,
    /** Kinect format calibration type */
    CameraCalibrationTypeKinect = 0
};

/**
 * @brief Frame type: color, depth, IR as well as confidence matrices.
 */ 
enum class FrameType: short {
    /** Color/BGR frame type */
    FrameTypeColor = 0,
    /** Int16 depth type in mm */
    FrameTypeDepth = 1,
    /** IR sensor frame type */
    FrameTypeIR = 2, // N.B. ilumination IR or thermic IR?
    /** Confidence levels */
    FrameTypeConfidence = 3,
    /** Human pose */
    FrameTypeHumanPose = 4
};

/**
 * @brief Codec parameters type. 
 */ 
enum class CodecParamsType: short {
    /** Default type */
    CodecParamsTypeDefault = -1,
    /** Libav codec configuration */
    CodecParamsTypeAv = 0,
    /** NvPipe configuration */
    CodecParamsTypeNvPipe = 1,
    /** ZDepth compression configuration */
    CodecParamsTypeZDepth = 2
};

/**
 * @brief SSP Message type. 
 */ 
enum class SSPMessageType: short {
    /** Default only */
    MessageTypeDefault = 0
};

/**
 * @brief Frame data type. This is a precise binary format information.
 */ 
enum class FrameDataType: short {
    // 0 for image frames, 

    /** Image frame */
    FrameDataTypeImageFrame = 0,
    
    // 1 for libav packets, 

    /** Libav packets  */
    FrameDataTypeLibavPackets = 1,
    
    // 2 for raw RGBA data, 

    /** Raw RGBA data */
    FrameDataTypeRawRGBA = 2,

    // 3 for raw GRAY16LE data,
    
    /** GRAY16LE data  */
    FrameDataTypeGRAY16LE = 3,

    // 4 for NvPipe packets, 
    
    /** NvPipe packet */ 
    FrameDataTypeNvPipePacket = 4, 

    // 5 for raw 32FC1 data, 

    /** Raw 32FC1 data */
    FrameDataTypeRaw32FC1 = 5,

    // 6 for YUV data,
    
    /** YUV data  */
    FrameDataTypeYUV = 6,

    // 7 for raw U8C1 data
    
    /** U8C1 data */
    FrameDataTypeU8C1 = 7,

    // 8 for object-human data
    FrameDataTypeObjectHumanData = 8
}; 

/**
 * @brief Sensor type: color or depth
 */ 
enum class SensorType: short {
    // 0 for color, 1 for depth

    /** Color sensor */
    SensorTypeColor = 0,

    /** Depth sensor */
    SensorTypeDepth = 1,

    /** IR sensor */
    SensorTypeIR = 2,
    
    /** Confidence */
    SensorTypeConfidence = 3
};

/**
 * @brief Camera calibration data
 */
struct CameraCalibrationStruct {
  // 0: Kinect parameters
  /**
   * Camera calibration type
   */
  CameraCalibrationType type = CameraCalibrationType::CameraCalibrationTypeDefault;
  /**
   * Opaque data blob #1
   */
  std::vector<unsigned char> data;
  /** 
   * Opaque data blob #2
   */
  std::vector<unsigned char> extra_data;

  /**
   * Default constructor
   */
  CameraCalibrationStruct() {}

  /**
   * Structure constructor
   * \param t camera calibration type
   * \param d opaque data blob #1
   * \param ed opaque data blob #2
   */
  CameraCalibrationStruct(CameraCalibrationType t, std::vector<unsigned char> d,
                          std::vector<unsigned char> ed)
      : type(t), data(d), extra_data(ed) {}

#ifndef __MOETSI_RAAS__
  template <class Archive> void serialize(Archive &ar) {
    ar(type, data, extra_data);
  }
#endif // !__MOETSI_RAAS__
};

/**
 * @brief Codec parameters
 */
struct CodecParamsStruct {
  // 0: av parameters, 1: nvPipe parameters, 2: zDepth parameters
  /**
   * Codec parameters type
   */
  CodecParamsType type = CodecParamsType::CodecParamsTypeDefault;
  /**
   * Opaque data blob #1
   */
  std::vector<unsigned char> data;
  /**
   * Opaque data blob #2
   */
  std::vector<unsigned char> extra_data;

  /** Default constructor */
  CodecParamsStruct() {}

  /**
   * Structural constructor
   * \param t codec type
   * \param d opaque data blob #1
   * \param ed opaque data blob #2
   */
  CodecParamsStruct(CodecParamsType t, std::vector<unsigned char> d,
                    std::vector<unsigned char> ed)
      : type(t), data(d), extra_data(ed) {}

#ifndef __MOETSI_RAAS__
  template <class Archive> void serialize(Archive &ar) {
    ar(type, data, extra_data);
  }
#endif // !__MOETSI_RAAS__
};

/**
 * @brief Frame struct: SSP frame.
 */ 
struct FrameStruct {

  // message id, currenly set to 0
  // This is to be used as "versioning", so if how messages are updated so that Sensor Stream Client
  // must interpret different "versions" of messages then this field will indicate the message version

  /**
   * SSP message type
   */
  SSPMessageType message_type;

  // 0 for color, 1 for depth, 2 for ir, 3 for confidence
  /**
   * Frame type
   */
  FrameType frame_type;

  /**
   * Frame data type
   */
  FrameDataType frame_data_type;

  /**
  * Random 16 char string that uniquely ids the frame stream.
  * Some decoders (like video) are stateful and so must keep track of streams.
  * This is automatically generated.
  */
  std::string stream_id;

  // frame binary data
  // We use a vector to know the size, basically a vector of bytes to store binary data

  /**
   * Frame binary data
   * We use a vector to know the size, basically a vector of bytes to store binary data
   */  
  std::vector<unsigned char> frame;

  /**
   * Codec info for video frames, null for image frames
   * Video decoder needs to know about the last receive frame
   * Requires to know the codec as well as additional parameters
   */  
  CodecParamsStruct codec_data;

  /**
   * Codec info for video frames, null for image frames
   */ 
  CameraCalibrationStruct camera_calibration_data;

  /**
   * Optional: scene description
   */
  std::string scene_desc;

  /**
   * Sensor id
   */
  unsigned int sensor_id;

  /**
   * Integer device id: distingish between devices in the same scene
   * Can be set by user.
   */
  unsigned int device_id;

  /**
   * Current frame number (increases over time)
   * Increases by 1 for each frame automatically when SSP server starts
   */
  unsigned int frame_id;

  /** 
   * Use for logging and timing to understand processing speeds.
   * Times are in ns
   */
  std::vector<uint64_t> timestamps;

#ifndef __MOETSI_RAAS__
  // Serialize method (not used by Server but is available)
  template <class Archive> void serialize(Archive &ar) {
    ar(message_type, frame_type, frame_data_type, stream_id, frame, codec_data,
       camera_calibration_data, scene_desc, sensor_id, device_id, frame_id,
       timestamps);
  }
#endif // !__MOETSI_RAAS__
};

#ifndef __MOETSI_RAAS__
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

#endif // !__MOETSI_RAAS__

} // namespace moetsi::ssp
