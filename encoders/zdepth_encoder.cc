//
// Created by amourao on 23-09-2019.
//

#include "zdepth_encoder.h"

ZDepthEncoder::ZDepthEncoder(int _fps) {
  fps_ = _fps;
  total_frame_counter_ = 0;
  frame_compressed_ = nullptr;
  frame_original_ = nullptr;
  codec_params_struct_ = nullptr;
  libav_decoder_ = nullptr;
  sws_context_ = nullptr;

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
    frame_compressed_->timestamps = std::vector<unsigned long>();
    frame_compressed_->timestamps.push_back(frame_original_->timestamps.front());
    frame_compressed_->timestamps.push_back(CurrentTimeMs());
    frame_compressed_->camera_calibration_data = fs->camera_calibration_data;

    uint16_t *data = nullptr;

    cv::Mat img;

    AVFrameSharedP pFrameO = nullptr;
    AVFrameSharedP pFrame = nullptr;

    if (fs->frame_data_type == 0) {

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

    } else if (fs->frame_data_type == 1) {

      if (libav_decoder_ == nullptr) {
        libav_decoder_ = std::make_unique<LibAvDecoder>();
        libav_decoder_->Init(getParams(*fs));
      }

      FrameStruct fss = *fs;
      img = libav_decoder_->Decode(fss);

      width_ = img.cols;
      height_ = img.rows;

      if (frame_compressed_->frame_type == 0) {
        cv::cvtColor(img, img, CV_BGR2BGRA);
      }

      data = reinterpret_cast<uint16_t *>(img.data);

    } else if (fs->frame_data_type == 2 || fs->frame_data_type == 3) {
      // fs->frame[8] ignores width and height set at [0] and [4] by
      // KinectReader
      memcpy(&width_, &fs->frame[0], sizeof(int));
      memcpy(&height_, &fs->frame[4], sizeof(int));
      data = reinterpret_cast<uint16_t *>(&fs->frame[8]);
    }

    if (codec_params_struct_ == nullptr)
      GetCodecParamsStruct();

    compressed_buffer_.clear();
    //TODO: send I Frame every X frames to allow decoder to catch up mid stream
    compressor_.Compress(width_, height_, data, compressed_buffer_,
                        total_frame_counter_ == 0);

    frame_compressed_->codec_data = *codec_params_struct_;
    frame_compressed_->frame = std::vector<unsigned char>(
        compressed_buffer_.data(), compressed_buffer_.data() + compressed_buffer_.size());
    frame_compressed_->timestamps.push_back(CurrentTimeMs());
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
    codec_params_struct_ = std::make_shared<CodecParamsStruct>();
    codec_params_struct_->type = 2;
    codec_params_struct_->data.resize(4 + 4);

    memcpy(&codec_params_struct_->data[0], &width_, sizeof(int));
    memcpy(&codec_params_struct_->data[4], &height_, sizeof(int));
  }
  return codec_params_struct_;
}

unsigned int ZDepthEncoder::GetFps() { return fps_; }
