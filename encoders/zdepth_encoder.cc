/**
 * \file zdepth_encoder.cc @brief ZDepth encoder
 */
// Created by amourao on 23-09-2019.
#include "zdepth_encoder.h"

namespace moetsi::ssp {

ZDepthEncoder::ZDepthEncoder(YAML::Node& _codec_parameters, int _fps) {
  fps_ = _fps;
  total_frame_counter_ = 0;
  frame_compressed_ = nullptr;
  frame_original_ = nullptr;
  codec_params_struct_ = nullptr;
  libav_decoder_ = nullptr;
  sws_context_ = nullptr;

  if (!_codec_parameters["send_I_frame_interval"].IsDefined()) {
      send_I_frame_interval_ = std::numeric_limits<unsigned int>::max();
      spdlog::warn("Missing key: \"send_I_frame_interval\", Using default: {}", send_I_frame_interval_);
  } else {
      send_I_frame_interval_ = _codec_parameters["send_I_frame_interval"].as<unsigned int>();
  }

  stream_id_ = RandomString(16);
}

ZDepthEncoder::~ZDepthEncoder() {

}

void ZDepthEncoder::AddFrameStruct(std::shared_ptr<FrameStruct> &fs) {
  frame_original_ = fs;

  if (frame_original_ == nullptr) {
    frame_compressed_ = nullptr;
  } else {

    if (frame_compressed_ == nullptr)
      frame_compressed_ = std::shared_ptr<FrameStruct>(new FrameStruct());

    frame_compressed_->device_id = fs->device_id;
    frame_compressed_->frame_data_type = FrameDataType::FrameDataTypeLibavPackets; // = 1;
    frame_compressed_->frame_id = total_frame_counter_;
    frame_compressed_->frame_type = fs->frame_type;
    frame_compressed_->message_type = fs->message_type;
    frame_compressed_->sensor_id = fs->sensor_id;
    frame_compressed_->stream_id = stream_id_;
    frame_compressed_->scene_desc = fs->scene_desc;
    frame_compressed_->timestamps.clear();
    frame_compressed_->timestamps = std::vector<uint64_t>();
    frame_compressed_->timestamps.push_back(frame_original_->timestamps.front());
    frame_compressed_->timestamps.push_back(CurrentTimeNs());
    frame_compressed_->camera_calibration_data = fs->camera_calibration_data;

    uint16_t *data = nullptr;

    cv::Mat img;

    AVFrameSharedP pFrameO = nullptr;
    AVFrameSharedP pFrame = nullptr;

    // FrameDataType::FrameDataTypeImageFrame = 0
    // if (fs->frame_data_type == 0) {
    if (fs->frame_data_type == FrameDataType::FrameDataTypeImageFrame) {      

      AVFrame *tmp = av_frame_alloc();
      pFrameO = std::shared_ptr<AVFrame>(tmp, AVFrameSharedDeleter);
      tmp = av_frame_alloc();
      pFrame = std::shared_ptr<AVFrame>(tmp, AVFrameSharedDeleter);

      image_decoder_.ImageBufferToAVFrame(fs, pFrameO);

      width_ = pFrameO->width;
      height_ = pFrameO->height;

      pFrame->format = AV_PIX_FMT_GBRP16LE;

      pFrame->width = width_;
      pFrame->height = height_;
      av_frame_get_buffer(pFrame.get(), 0);

      if (sws_context_ == nullptr)
        sws_context_ = std::unique_ptr<SwsContext, SwsContextDeleter>(sws_getContext(width_, height_, (AVPixelFormat)pFrameO->format,
                                 width_, height_, (AVPixelFormat)pFrame->format,
                                 SWS_BILINEAR, NULL, NULL, NULL));

      sws_scale(sws_context_.get(), (const uint8_t *const *)pFrameO->data,
                pFrameO->linesize, 0, pFrameO->height, pFrame->data,
                pFrame->linesize);

      data = reinterpret_cast<uint16_t *>(&pFrame->data[0][0]);

    // FrameDataType::FrameDataTypeLibavPackets = 1
    // } else if (fs->frame_data_type == 1) {
    } else if (fs->frame_data_type == FrameDataType::FrameDataTypeLibavPackets) {

      if (libav_decoder_ == nullptr) {
        libav_decoder_ = std::unique_ptr<LibAvDecoder>(new LibAvDecoder());
        libav_decoder_->Init(getParams(*fs));
      }

      FrameStruct fss = *fs;
      img = libav_decoder_->Decode(fss);

      width_ = img.cols;
      height_ = img.rows;

      // FrameType::FrameTypeColor = 0
      // if (frame_compressed_->frame_type == 0) {
      if (frame_compressed_->frame_type == FrameType::FrameTypeColor) {
        cv::cvtColor(img, img, CV_BGR2BGRA);
      }

      data = reinterpret_cast<uint16_t *>(img.data);

    // FrameDataType::FrameDataTypeRawRGBA = 2
    // FrameDataType::FrameDataTypeGRAY16LE = 3
    // } else if (fs->frame_data_type == 2 || fs->frame_data_type == 3) {
    } else if (fs->frame_data_type == FrameDataType::FrameDataTypeRawRGBA || fs->frame_data_type == FrameDataType::FrameDataTypeGRAY16LE) {
      // fs->frame[8] ignores width and height set at [0] and [4] by
      // KinectReader
      memcpy(&width_, &fs->frame[0], sizeof(int));
      memcpy(&height_, &fs->frame[4], sizeof(int));
      data = reinterpret_cast<uint16_t *>(&fs->frame[8]);
    } else
      spdlog::error("zdepth: Unsupported frame_data_type {}", fs->frame_data_type);

    if (codec_params_struct_ == nullptr)
      GetCodecParamsStruct();

    compressed_buffer_.clear();
    //TODO: send I Frame every X frames to allow decoder to catch up mid stream
    compressor_.Compress(width_, height_, data, compressed_buffer_,
                        (total_frame_counter_ % send_I_frame_interval_) == 0 || total_frame_counter_ == 0);

    frame_compressed_->codec_data = *codec_params_struct_;
    frame_compressed_->frame = std::vector<unsigned char>(
        compressed_buffer_.data(), compressed_buffer_.data() + compressed_buffer_.size());
    frame_compressed_->timestamps.push_back(CurrentTimeNs());
    total_frame_counter_++;

  }
}

void ZDepthEncoder::NextPacket() {
  frame_original_ = nullptr;
  frame_compressed_ = nullptr;
}

bool ZDepthEncoder::HasNextPacket() { return true; }

std::shared_ptr<FrameStruct> ZDepthEncoder::CurrentFrameEncoded() {
  return frame_compressed_;
}

std::shared_ptr<FrameStruct> ZDepthEncoder::CurrentFrameOriginal() {
  return frame_original_;
}

std::shared_ptr<CodecParamsStruct> ZDepthEncoder::GetCodecParamsStruct() {
  if (codec_params_struct_ == NULL) {
    codec_params_struct_ = std::shared_ptr<CodecParamsStruct>(new CodecParamsStruct());
    codec_params_struct_->type = CodecParamsType::CodecParamsTypeZDepth; // 2;
    codec_params_struct_->data.resize(4 + 4);

    memcpy(&codec_params_struct_->data[0], &width_, sizeof(int));
    memcpy(&codec_params_struct_->data[4], &height_, sizeof(int));
  }
  return codec_params_struct_;
}

unsigned int ZDepthEncoder::GetFps() { return fps_; }

} // namespace moetsi::ssp
