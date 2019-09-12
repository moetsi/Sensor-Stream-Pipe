//
// Created by amourao on 11-09-2019.
//

#include <unistd.h>

#include <k4a/k4a.h>

#include <ctime>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <thread>

#include <yaml-cpp/yaml.h>

//#include <opencv2/imgproc.hpp>

#include "../readers/KinectReader.h"
#include "../structs/FrameStruct.hpp"
#include "../utils/KinectUtils.h"
#include "../utils/Utils.h"
#include "NvEncoder.h"

NvEncoder::NvEncoder(YAML::Node _codec_parameters, uint _fps) {
  buildEncoder(_codec_parameters, _fps);
  fps = _fps;
  int bufferSize = width * height * 4;
  compressed.resize(bufferSize);
  paramsStruct = nullptr;
  frameCompressed = nullptr;
  frameOriginal = nullptr;
  getCodecParamsStruct();
}

NvEncoder::~NvEncoder() {}

void NvEncoder::addFrameStruct(FrameStruct *fs) {
  frameOriginal = fs;

  if (frameOriginal == nullptr) {
    frameCompressed = nullptr;
  } else {

    if (frameCompressed == nullptr)
      frameCompressed = new FrameStruct();
    frameCompressed->codec_data = *paramsStruct;
    frameCompressed->deviceId = fs->deviceId;
    frameCompressed->frameDataType = 4;
    frameCompressed->frameId = totalCurrentFrameCounter;
    frameCompressed->frameType = fs->frameType;
    frameCompressed->messageType = fs->messageType;
    frameCompressed->sensorId = fs->sensorId;
    frameCompressed->sceneDesc = fs->sceneDesc;

    uint64_t srcPitch = width;

    if (frameCompressed->frameType == 0) {
      srcPitch *= 4;
    } else {
      srcPitch *= 1;
    }

    uint64_t compressedSize =
        NvPipe_Encode(encoder, &fs->frame[0], srcPitch, compressed.data(),
                      compressed.size(), width, height, false);

    frameCompressed->frame.clear();
    frameCompressed->frame = std::vector<unsigned char>(
        compressed.data(), compressed.data() + compressedSize);

    totalCurrentFrameCounter++;
  }
}

void NvEncoder::nextPacket() {
  frameOriginal = nullptr;
  frameCompressed = nullptr;
}

bool NvEncoder::hasNextPacket() { return frameCompressed != nullptr; }

FrameStruct *NvEncoder::currentFrameEncoded() { return frameCompressed; }

FrameStruct *NvEncoder::currentFrameOriginal() { return frameOriginal; }

CodecParamsStruct *NvEncoder::getCodecParamsStruct() {
  if (paramsStruct == NULL) {
    paramsStruct = new CodecParamsStruct();
    paramsStruct->type = 1;
    paramsStruct->data.resize(4 + 4 + 1 + 1);

    memcpy(&paramsStruct->data[0], &width, sizeof(int));
    memcpy(&paramsStruct->data[4], &height, sizeof(int));

    ushort format_ushort, codec_ushort;

    if (format == NVPIPE_RGBA32) {
      format_ushort = 0;
    } else if (format == NVPIPE_UINT4) {
      format_ushort = 1;
    } else if (format == NVPIPE_UINT8) {
      format_ushort = 2;
    } else if (format == NVPIPE_UINT16) {
      format_ushort = 3;
    } else if (format == NVPIPE_UINT32) {
      format_ushort = 4;
    }
    paramsStruct->data[8] = (uchar)format_ushort;

    if (codec == NVPIPE_H264) {
      codec_ushort = 0;
    } else if (codec == NVPIPE_HEVC) {
      codec_ushort = 1;
    }

    paramsStruct->data[9] = (uchar)codec_ushort;
  }
}

uint NvEncoder::getFps() { return fps; }

void NvEncoder::buildEncoder(YAML::Node config, uint fps) {

  if (!config["codec_name"].IsDefined()) {
    std::cout << "WARNING: Missing key: \"codec_name\"" << std::endl;
    std::cout << "Using default: NVPIPE_H264" << std::endl;
    codec = NVPIPE_H264;
  } else {
    std::string codec_str = config["codec_name"].as<std::string>();
    if (codec_str == "NVPIPE_H264") {
      codec = NVPIPE_H264;
    } else if (codec_str == "NVPIPE_HEVC") {
      codec = NVPIPE_HEVC;
      ;
    } else {
      std::cerr << "Invalid value for: \"codec_name\": " << codec_str
                << std::endl;
      std::cerr << "Supported values are NVPIPE_H264 and "
                   "NVPIPE_HEVC"
                << std::endl;
      throw "Invalid value for: \"codec_name\"";
    }
  }

  if (!config["compression"].IsDefined()) {
    std::cout << "WARNING: Missing key: \"compression\"" << std::endl;
    std::cout << "Using default: NVPIPE_LOSSY" << std::endl;
    compression = NVPIPE_LOSSY;
  } else {
    std::string compression_str = config["compression"].as<std::string>();
    if (compression_str == "NVPIPE_LOSSY") {
      compression = NVPIPE_LOSSY;
    } else if (compression_str == "NVPIPE_LOSSLESS") {
      compression = NVPIPE_LOSSLESS;
      ;
    } else {
      std::cerr << "Invalid value for: \"compression\": " << compression_str
                << std::endl;
      std::cerr << "Supported values are NVPIPE_LOSSY and "
                   "NVPIPE_LOSSLESS"
                << std::endl;
      throw "Invalid value for: \"compression\"";
    }
  }

  if (!config["input_format"].IsDefined()) {
    std::cout << "Missing key: \"input_format\"" << std::endl;
    std::cerr << "Supported values are NVPIPE_RGBA32, NVPIPE_UINT4, "
                 "NVPIPE_UINT8, NVPIPE_UINT16 and NVPIPE_UINT32"
              << std::endl;
    throw "Invalid value for: \"input_format\"";
  } else {
    std::string input_format_str = config["input_format"].as<std::string>();
    if (input_format_str == "NVPIPE_RGBA32") {
      format = NVPIPE_RGBA32;
    } else if (input_format_str == "NVPIPE_UINT4") {
      format = NVPIPE_UINT4;
    } else if (input_format_str == "NVPIPE_UINT8") {
      format = NVPIPE_UINT8;
    } else if (input_format_str == "NVPIPE_UINT16") {
      format = NVPIPE_UINT16;
    } else if (input_format_str == "NVPIPE_UINT32") {
      format = NVPIPE_UINT32;
    } else {
      std::cerr << "Invalid value for: \"input_format\": " << input_format_str
                << std::endl;
      std::cerr << "Supported values are NVPIPE_RGBA32, NVPIPE_UINT4, "
                   "NVPIPE_UINT8, NVPIPE_UINT16 and NVPIPE_UINT32"
                << std::endl;
      throw "Invalid value for: \"input_format\"";
    }
  }

  uint bitrate = 8 * 1000 * 1000;
  if (config["bit_rate"].IsDefined()) {
    bitrate = config["bit_rate"].as<uint>();
  } else {
    std::cout << "Using default bit_rate = " << bitrate << std::endl;
  }

  if (config["width"].IsDefined()) {
    width = config["width"].as<uint>();
  } else {
    throw "Invalid value for: \"width\"";
  }

  if (config["height"].IsDefined()) {
    height = config["height"].as<uint>();
  } else {
    throw "Invalid value for: \"height\"";
  }

  encoder = NvPipe_CreateEncoder(format, codec, compression, bitrate, fps,
                                 width, height);
}
