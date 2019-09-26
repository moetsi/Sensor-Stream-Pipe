//
// Created by amourao on 11-09-2019.
//

#include <unistd.h>

#include "../utils/Logger.h"
#include <k4a/k4a.h>

#include <ctime>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <thread>

#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

//#include <opencv2/imgproc.hpp>

#include "NvEncoder.h"

NvEncoder::NvEncoder(YAML::Node _codec_parameters, uint _fps) {
  buildEncoder(_codec_parameters);
  fps = _fps;
  totalCurrentFrameCounter = 0;
  paramsStruct = nullptr;
  frameCompressed = nullptr;
  frameOriginal = nullptr;
  encoder = nullptr;
  sws_ctx = nullptr;
  fd = nullptr;

  stream_id = randomString(16);
}

NvEncoder::~NvEncoder() {}

void NvEncoder::addFrameStruct(FrameStruct *fs) {

  frameOriginal = fs;

  if (frameOriginal == nullptr) {
    frameCompressed = nullptr;
  } else {

    if (frameCompressed == nullptr)
      frameCompressed = new FrameStruct();

    frameCompressed->deviceId = fs->deviceId;
    frameCompressed->frameDataType = 1;
    frameCompressed->frameId = totalCurrentFrameCounter;
    frameCompressed->frameType = fs->frameType;
    frameCompressed->messageType = fs->messageType;
    frameCompressed->sensorId = fs->sensorId;
    frameCompressed->streamId = stream_id;
    frameCompressed->sceneDesc = fs->sceneDesc;
    frameCompressed->timestamps.clear();
    frameCompressed->timestamps.push_back(fs->timestamps.front());
    frameCompressed->timestamps.push_back(currentTimeMs());

    char *data = nullptr;

    AVFrame *pFrameO = nullptr;
    AVFrame *pFrame = nullptr;
    cv::Mat img;

    uint64_t compressed_size = 0;

    if (fs->frameDataType == 0) {

      pFrameO = av_frame_alloc();
      pFrame = av_frame_alloc();

      id.imageBufferToAVFrame(fs, pFrameO);

      width = pFrameO->width;
      height = pFrameO->height;

      if (fs->frameType == 0) {
        pFrame->format = AV_PIX_FMT_BGRA;
      } else {
        pFrame->format = AV_PIX_FMT_GBRP16LE;
      }

      pFrame->width = width;
      pFrame->height = height;
      av_frame_get_buffer(pFrame, 0);

      if (sws_ctx == nullptr)
        sws_ctx = sws_getContext(width, height, (AVPixelFormat)pFrameO->format,
                                 width, height, (AVPixelFormat)pFrame->format,
                                 SWS_BILINEAR, NULL, NULL, NULL);

      sws_scale(sws_ctx, (const uint8_t *const *)pFrameO->data,
                pFrameO->linesize, 0, pFrameO->height, pFrame->data,
                pFrame->linesize);

      data = reinterpret_cast<char *>(&pFrame->data[0][0]);

    } else if (fs->frameDataType == 1) {

      if (fd == nullptr) {
        fd = new LibAvDecoder();
        fd->init(fs->codec_data.getParams());
      }

      img = fd->decode(fs);

      width = img.cols;
      height = img.rows;

      if (frameCompressed->frameType == 0) {
        cv::cvtColor(img, img, CV_BGR2BGRA);
      }

      data = reinterpret_cast<char *>(img.data);

    } else if (fs->frameDataType == 2 || fs->frameDataType == 3) {
      // fs->frame[8] ignores width and height set at [0] and [4] by
      // KinectReader
      memcpy(&width, &fs->frame[0], sizeof(int));
      memcpy(&height, &fs->frame[4], sizeof(int));
      data = reinterpret_cast<char *>(&fs->frame[8]);
    }

    if (encoder == nullptr) {
      getCodecParamsStruct();

      if (frameCompressed->frameType == 0) {
        encoder = NvPipe_CreateEncoder(format, codec, compression, bitrate, fps,
                                       width, height);
      } else {
        // For some reason, NVPipe rebuilds the enconder if the frame size
        // "changes". By using the expected width here, we avoid having the
        // program crash due an invalid free on the encoder
        encoder = NvPipe_CreateEncoder(format, codec, compression, bitrate, fps,
                                       width * 2, height);
      }
    }

    uint64_t srcPitch = width;

    if (frameCompressed->frameType == 0) {
      srcPitch *= 4;
    } else {
      srcPitch *= 2;
    }

    if (encoder == nullptr) {
      spdlog::error("Could not create new NVEncoder");
      spdlog::error(NvPipe_GetError(NULL));
      spdlog::error("Did you reach the number of parallel encoding sessions "
                    "available? (2 on non-Quadro cards)");
      exit(1);
    }
    compressed_size = NvPipe_Encode(encoder, data, srcPitch, compressed.data(),
                                    compressed.size(), width, height, false);

    frameCompressed->codec_data = *paramsStruct;
    frameCompressed->frame = std::vector<unsigned char>(
        compressed.data(), compressed.data() + compressed_size);
    frameCompressed->timestamps.push_back(currentTimeMs());
    totalCurrentFrameCounter++;

    if (pFrame != nullptr) {
      av_frame_free(&pFrame);
      av_frame_free(&pFrameO);
    }
  }
}

void NvEncoder::nextPacket() {
  if (frameOriginal != nullptr)
    delete frameOriginal;
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

    int bufferSize = width * height * 4;
    compressed.resize(bufferSize);

    memcpy(&paramsStruct->data[0], &width, sizeof(int));
    memcpy(&paramsStruct->data[4], &height, sizeof(int));

    ushort format_ushort = 0, codec_ushort = 0;

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
  return paramsStruct;
}

uint NvEncoder::getFps() { return fps; }

void NvEncoder::buildEncoder(YAML::Node config) {

  if (!config["codec_name"].IsDefined()) {
    spdlog::warn("Missing key: \"codec_name\", Using default: NVPIPE_H264");
    codec = NVPIPE_H264;
  } else {
    std::string codec_str = config["codec_name"].as<std::string>();
    if (codec_str == "NVPIPE_H264") {
      codec = NVPIPE_H264;
    } else if (codec_str == "NVPIPE_HEVC") {
      codec = NVPIPE_HEVC;
      ;
    } else {
      spdlog::error("Invalid value for: \"codec_name\": {}, Supported values "
                    "are NVPIPE_H264 and NVPIPE_HEVC",
                    codec_str);
      throw "Invalid value for: \"codec_name\"";
    }
  }

  if (!config["compression"].IsDefined()) {
    spdlog::warn("Missing key: \"compression\", Using default: NVPIPE_LOSSY");
    compression = NVPIPE_LOSSY;
  } else {
    std::string compression_str = config["compression"].as<std::string>();
    if (compression_str == "NVPIPE_LOSSY") {
      compression = NVPIPE_LOSSY;
    } else if (compression_str == "NVPIPE_LOSSLESS") {
      compression = NVPIPE_LOSSLESS;
      ;
    } else {
      spdlog::error("Invalid value for: \"compression\": {}, Supported values "
                    "are NVPIPE_LOSSY and NVPIPE_LOSSLESS",
                    compression_str);
      throw "Invalid value for: \"compression\"";
    }
  }

  if (!config["input_format"].IsDefined()) {
    spdlog::error("Missing value for: \"input_format\", Supported values are "
                  "NVPIPE_RGBA32, NVPIPE_UINT4, NVPIPE_UINT8, NVPIPE_UINT16 "
                  "and NVPIPE_UINT32");
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
      spdlog::error("Invalid value for: \"input_format\": {}, Supported values "
                    "are NVPIPE_RGBA32, NVPIPE_UINT4, NVPIPE_UINT8, "
                    "NVPIPE_UINT16 and NVPIPE_UINT32",
                    input_format_str);
      throw "Invalid value for: \"input_format\"";
    }
  }

  bitrate = 8 * 1000 * 1000;
  if (config["bit_rate"].IsDefined()) {
    bitrate = config["bit_rate"].as<uint>();
  } else {
    spdlog::warn("Missing key: \"bit_rate\", Using default: {}", bitrate);
  }
}
