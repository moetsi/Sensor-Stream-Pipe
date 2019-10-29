//
// Created by amourao on 11-09-2019.
//

#include <unistd.h>

#include "../utils/logger.h"

#include <ctime>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <thread>

#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

//#include <opencv2/imgproc.hpp>

#include "nv_encoder.h"

NvEncoder::NvEncoder(YAML::Node _codec_parameters, unsigned int _fps) {
  BuildEncoder(_codec_parameters);
  fps_ = _fps;
  total_frame_counter_ = 0;
  codec_params_struct_ = nullptr;
  frame_compressed_ = nullptr;
  frame_original_ = nullptr;
  encoder_ = nullptr;
  sws_context_ = nullptr;
  lib_av_decoder_ = nullptr;

  stream_id_ = RandomString(16);
}

NvEncoder::~NvEncoder() {}

void NvEncoder::AddFrameStruct(std::shared_ptr<FrameStruct> &fs) {

  frame_original_ = fs;

  if (frame_original_ == nullptr) {
    frame_compressed_ = nullptr;
  } else {

    if (frame_compressed_ == nullptr)
      frame_compressed_ = std::make_shared<FrameStruct>();

    frame_compressed_->device_id = fs->device_id;
    frame_compressed_->frame_data_type = 1;
    frame_compressed_->frame_id = total_frame_counter_;
    frame_compressed_->frame_type = fs->frame_type;
    frame_compressed_->message_type = fs->message_type;
    frame_compressed_->sensor_id = fs->sensor_id;
    frame_compressed_->stream_id = stream_id_;
    frame_compressed_->scene_desc = fs->scene_desc;
    frame_compressed_->timestamps.clear();
    frame_compressed_->timestamps.push_back(fs->timestamps.front());
    frame_compressed_->timestamps.push_back(CurrentTimeMs());
    frame_compressed_->camera_calibration_data = fs->camera_calibration_data;

    char *data = nullptr;

    AVFrameSharedP frame_av_original = nullptr;
    AVFrameSharedP frame_av_encoded = nullptr;
    cv::Mat img;

    uint64_t compressed_size = 0;

    if (fs->frame_data_type == 0) {

      frame_av_original =
          std::shared_ptr<AVFrame>(av_frame_alloc(), AVFrameSharedDeleter);
      frame_av_encoded =
          std::shared_ptr<AVFrame>(av_frame_alloc(), AVFrameSharedDeleter);

      image_decoder_.ImageBufferToAVFrame(fs, frame_av_original);

      width_ = frame_av_original->width;
      height_ = frame_av_original->height;

      if (fs->frame_type == 0) {
        frame_av_encoded->format = AV_PIX_FMT_BGRA;
      } else {
        frame_av_encoded->format = AV_PIX_FMT_GBRP16LE;
      }

      frame_av_encoded->width = width_;
      frame_av_encoded->height = height_;
      av_frame_get_buffer(frame_av_encoded.get(), 0);

      if (sws_context_ == nullptr)
        sws_context_ = std::unique_ptr<struct SwsContext, SwsContextDeleter>(
            sws_getContext(width_, height_,
                           (AVPixelFormat)frame_av_original->format, width_,
                           height_, (AVPixelFormat)frame_av_encoded->format,
                           SWS_BILINEAR, NULL, NULL, NULL));

      sws_scale(sws_context_.get(),
                (const uint8_t *const *)frame_av_original->data,
                frame_av_original->linesize, 0, frame_av_original->height,
                frame_av_encoded->data, frame_av_encoded->linesize);

      data = reinterpret_cast<char *>(&frame_av_encoded->data[0][0]);

    } else if (fs->frame_data_type == 1) {

      if (lib_av_decoder_ == nullptr) {
        lib_av_decoder_ = std::make_unique<LibAvDecoder>();
        lib_av_decoder_->Init(getParams(*fs));
      }

      img = lib_av_decoder_->Decode(*fs);

      width_ = img.cols;
      height_ = img.rows;

      if (frame_compressed_->frame_type == 0) {
        cv::cvtColor(img, img, CV_BGR2BGRA);
      }

      data = reinterpret_cast<char *>(img.data);

    } else if (fs->frame_data_type == 2 || fs->frame_data_type == 3) {
      // fs->frame[8] ignores width and height set at [0] and [4] by
      // KinectReader
      memcpy(&width_, &fs->frame[0], sizeof(int));
      memcpy(&height_, &fs->frame[4], sizeof(int));
      data = reinterpret_cast<char *>(&fs->frame[8]);
    }

    if (encoder_ == nullptr) {
      GetCodecParamsStruct();

      if (frame_compressed_->frame_type == 0) {
        encoder_ = std::unique_ptr<NvPipe, NVPipeDeleter>(NvPipe_CreateEncoder(
            format_, codec_, compression_, bitrate_, fps_, width_, height_));
      } else {
        // For some reason, NVPipe rebuilds the enconder if the frame size
        // "changes". By using the expected width here, we avoid having the
        // program crash due an invalid free on the encoder
        encoder_ = std::unique_ptr<NvPipe, NVPipeDeleter>(
            NvPipe_CreateEncoder(format_, codec_, compression_, bitrate_, fps_,
                                 width_ * 2, height_));
      }
    }

    uint64_t src_pitch = width_;

    if (frame_compressed_->frame_type == 0) {
      src_pitch *= 4;
    } else {
      src_pitch *= 2;
    }

    if (encoder_ == nullptr) {
      spdlog::error("Could not create new NVEncoder");
      spdlog::error(NvPipe_GetError(NULL));
      spdlog::error("Did you reach the number of parallel encoding sessions "
                    "available? (2 on non-Quadro cards)");
      exit(1);
    }

    // TODO: send I Frame every X frames to allow decoder to catch up mid stream
    compressed_size = NvPipe_Encode(
        encoder_.get(), data, src_pitch, compressed_buffer_.data(),
        compressed_buffer_.size(), width_, height_, false);

    if (compressed_size == 0) {
      spdlog::error("Could not encode frame on NVEncoder");
      spdlog::error(NvPipe_GetError(encoder_.get()));
      exit(1);
    }

    frame_compressed_->codec_data = *codec_params_struct_;
    frame_compressed_->frame = std::vector<unsigned char>(
        compressed_buffer_.data(), compressed_buffer_.data() + compressed_size);
    frame_compressed_->timestamps.push_back(CurrentTimeMs());
    total_frame_counter_++;
  }
}

void NvEncoder::NextPacket() {
  frame_original_ = nullptr;
  frame_compressed_ = nullptr;
}

bool NvEncoder::HasNextPacket() { return frame_compressed_ != nullptr; }

std::shared_ptr<FrameStruct> NvEncoder::CurrentFrameEncoded() {
  return frame_compressed_;
}

std::shared_ptr<FrameStruct> NvEncoder::CurrentFrameOriginal() {
  return frame_original_;
}

std::shared_ptr<CodecParamsStruct> NvEncoder::GetCodecParamsStruct() {
  if (codec_params_struct_ == NULL) {
    codec_params_struct_ = std::make_shared<CodecParamsStruct>();
    codec_params_struct_->type = 1;
    codec_params_struct_->data.resize(4 + 4 + 1 + 1);

    int bufferSize = width_ * height_ * 4;
    compressed_buffer_.resize(bufferSize);

    memcpy(&codec_params_struct_->data[0], &width_, sizeof(int));
    memcpy(&codec_params_struct_->data[4], &height_, sizeof(int));

    ushort format_ushort = 0, codec_ushort = 0;

    if (format_ == NVPIPE_RGBA32) {
      format_ushort = 0;
    } else if (format_ == NVPIPE_UINT4) {
      format_ushort = 1;
    } else if (format_ == NVPIPE_UINT8) {
      format_ushort = 2;
    } else if (format_ == NVPIPE_UINT16) {
      format_ushort = 3;
    } else if (format_ == NVPIPE_UINT32) {
      format_ushort = 4;
    }

    codec_params_struct_->data[8] = (uchar)format_ushort;

    if (codec_ == NVPIPE_H264) {
      codec_ushort = 0;
    } else if (codec_ == NVPIPE_HEVC) {
      codec_ushort = 1;
    }

    codec_params_struct_->data[9] = (uchar)codec_ushort;
  }
  return codec_params_struct_;
}

unsigned int NvEncoder::GetFps() { return fps_; }

void NvEncoder::BuildEncoder(YAML::Node _codec_parameters) {

  if (!_codec_parameters["codec_name"].IsDefined()) {
    spdlog::warn("Missing key: \"codec_name\", Using default: NVPIPE_H264");
    codec_ = NVPIPE_H264;
  } else {
    std::string codec_str = _codec_parameters["codec_name"].as<std::string>();
    if (codec_str == "NVPIPE_H264") {
      codec_ = NVPIPE_H264;
    } else if (codec_str == "NVPIPE_HEVC") {
      codec_ = NVPIPE_HEVC;
      ;
    } else {
      spdlog::error("Invalid value for: \"codec_name\": {}, Supported values "
                    "are NVPIPE_H264 and NVPIPE_HEVC",
                    codec_str);
      throw "Invalid value for: \"codec_name\"";
    }
  }

  if (!_codec_parameters["compression"].IsDefined()) {
    spdlog::warn("Missing key: \"compression\", Using default: NVPIPE_LOSSY");
    compression_ = NVPIPE_LOSSY;
  } else {
    std::string compression_str =
        _codec_parameters["compression"].as<std::string>();
    if (compression_str == "NVPIPE_LOSSY") {
      compression_ = NVPIPE_LOSSY;
    } else if (compression_str == "NVPIPE_LOSSLESS") {
      compression_ = NVPIPE_LOSSLESS;
      ;
    } else {
      spdlog::error("Invalid value for: \"compression\": {}, Supported values "
                    "are NVPIPE_LOSSY and NVPIPE_LOSSLESS",
                    compression_str);
      throw "Invalid value for: \"compression\"";
    }
  }

  if (!_codec_parameters["input_format"].IsDefined()) {
    spdlog::error("Missing value for: \"input_format\", Supported values are "
                  "NVPIPE_RGBA32, NVPIPE_UINT4, NVPIPE_UINT8, NVPIPE_UINT16 "
                  "and NVPIPE_UINT32");
    throw "Invalid value for: \"input_format\"";
  } else {
    std::string input_format_str =
        _codec_parameters["input_format"].as<std::string>();
    if (input_format_str == "NVPIPE_RGBA32") {
      format_ = NVPIPE_RGBA32;
    } else if (input_format_str == "NVPIPE_UINT4") {
      format_ = NVPIPE_UINT4;
    } else if (input_format_str == "NVPIPE_UINT8") {
      format_ = NVPIPE_UINT8;
    } else if (input_format_str == "NVPIPE_UINT16") {
      format_ = NVPIPE_UINT16;
    } else if (input_format_str == "NVPIPE_UINT32") {
      format_ = NVPIPE_UINT32;
    } else {
      spdlog::error("Invalid value for: \"input_format\": {}, Supported values "
                    "are NVPIPE_RGBA32, NVPIPE_UINT4, NVPIPE_UINT8, "
                    "NVPIPE_UINT16 and NVPIPE_UINT32",
                    input_format_str);
      throw "Invalid value for: \"input_format\"";
    }
  }

  bitrate_ = 8 * 1000 * 1000;
  if (_codec_parameters["bit_rate"].IsDefined()) {
    bitrate_ = _codec_parameters["bit_rate"].as<unsigned int>();
  } else {
    spdlog::warn("Missing key: \"bit_rate\", Using default: {}", bitrate_);
  }
}
