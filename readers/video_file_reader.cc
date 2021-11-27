/**
 * \file video_file_reader.cc @brief Video file reader
 */
// Created by amourao on 26-06-2019.
#include "video_file_reader.h"

namespace moetsi::ssp {

VideoFileReader::VideoFileReader(std::string &filename) {
  current_frame_counter_ = 0;
  eof_reached_ = false;
  libav_ready_ = false;
  filename_ = filename;
  video_stream_indexes_from_file_ = false;
}

VideoFileReader::VideoFileReader(
    std::string &filename, std::vector<unsigned int> &_video_stream_indexes) {
  current_frame_counter_ = 0;
  eof_reached_ = false;
  libav_ready_ = false;
  filename_ = filename;
  video_stream_indexes_ = _video_stream_indexes;
  video_stream_indexes_from_file_ = true;
}

VideoFileReader::~VideoFileReader() {
  av_packet_unref(packet_);
  av_packet_free(&packet_);

  for (auto const &x : av_codec_contexts_) {
    AVCodecContext *c = x.second;
    avcodec_free_context(&c);
  }

  avformat_close_input(&av_format_context_);

}

void VideoFileReader::Init(std::string &filename) {
  camera_calibration_struct_ = nullptr;
  //av_register_all();
  spdlog::info("VideoFileReader: initializing all the containers, codecs and "
               "protocols.");

  av_format_context_ = avformat_alloc_context();
  if (!av_format_context_) {
    spdlog::error("Could not allocate memory for Format Context.");
    exit(1);
  }

  spdlog::info(
      "Opening the input file and loading format (container) header {}",
      filename);

  if (avformat_open_input(&av_format_context_, filename.c_str(), NULL, NULL) !=
      0) {
    spdlog::error("Could not open the file.");
    exit(1);
  }

  spdlog::info("format {}, duration {} us, bit_rate {}",
               av_format_context_->iformat->name, av_format_context_->duration,
               av_format_context_->bit_rate);

  spdlog::info("Finding stream info from format.");

  if (avformat_find_stream_info(av_format_context_, NULL) < 0) {
    spdlog::error("Could not get the stream info.");
    exit(-1);
  }

  camera_calibration_struct_ = std::shared_ptr<CameraCalibrationStruct>(new CameraCalibrationStruct());
  camera_calibration_struct_->type = CameraCalibrationType::CameraCalibrationTypeKinect; // 0;
  camera_calibration_struct_->extra_data.resize(2);
  // the component that knows how to enCOde and DECode the stream
  // it's the codec (audio or video)
  // http://ffmpeg.org/doxygen/trunk/structAVCodec.html

  // this component describes the properties of a codec used by the stream i
  // https://ffmpeg.org/doxygen/trunk/structAVCodecParameters.html

  // loop though all the streams and print its main information
  for (unsigned int i = 0; i < av_format_context_->nb_streams; i++) {
    AVCodecParameters *codec_parameter =
        av_format_context_->streams[i]->codecpar;
    spdlog::info("AVStream->time_base before open coded: {}/{}",
                 av_format_context_->streams[i]->time_base.num,
                 av_format_context_->streams[i]->time_base.den);
    spdlog::info("AVStream->r_frame_rate before open coded: {}/{}",
                 av_format_context_->streams[i]->r_frame_rate.num,
                 av_format_context_->streams[i]->r_frame_rate.den);

    spdlog::info("AVStream->start_time: {}",
                 av_format_context_->streams[i]->start_time);
    spdlog::info("AVStream->duration: {}",
                 av_format_context_->streams[i]->duration);

    spdlog::info("finding the proper decoder (CODEC)",
                 av_format_context_->streams[i]->duration);

    AVCodec *codec = const_cast<AVCodec*>(avcodec_find_decoder(codec_parameter->codec_id));
    if (codec == NULL) {
      spdlog::warn("Non video stream detected ({}), skipping", i);
      if (av_format_context_->streams[i]->codecpar->extradata_size) {
        // CameraCalibrationType::CameraCalibrationTypeDefault=-1
        if (camera_calibration_struct_->type == CameraCalibrationType::CameraCalibrationTypeDefault) {
          camera_calibration_struct_->type = CameraCalibrationType::CameraCalibrationTypeKinect; // = 0;
          camera_calibration_struct_->extra_data.resize(2);
        }
        camera_calibration_struct_->data = std::vector<unsigned char>(
            av_format_context_->streams[i]->codecpar->extradata,
            av_format_context_->streams[i]->codecpar->extradata +
                av_format_context_->streams[i]->codecpar->extradata_size + 1);
      }
    } else if (codec_parameter->codec_type == AVMEDIA_TYPE_VIDEO) {
      spdlog::warn("Video stream detected ({})", i);
      if (!video_stream_indexes_from_file_)
        video_stream_indexes_.push_back(i);

      std::vector<unsigned int>::iterator it;

      it = find(video_stream_indexes_.begin(), video_stream_indexes_.end(), i);
      if (it != video_stream_indexes_.end()) {

        spdlog::info("Video Codec: resolution {}x{}", codec_parameter->width,
                     codec_parameter->height);

        AVCodecContext *codec_context = avcodec_alloc_context3(codec);
        if (!codec_context) {
          spdlog::error("Failed to allocated memory for AVCodecContext.");
          exit(-1);
        }

        if (avcodec_parameters_to_context(codec_context, codec_parameter) < 0) {
          spdlog::error("Failed to copy codec params to codec context.");
          exit(-1);
        }

        if (avcodec_open2(codec_context, codec, NULL) < 0) {
          spdlog::error("Failed to open codec through avcodec_open2.");
          exit(-1);
        }

        AVDictionary *metadata = av_format_context_->streams[i]->metadata;
        AVDictionaryEntry *t = nullptr;
        while ((t = av_dict_get(metadata, "", t, AV_DICT_IGNORE_SUFFIX))) {
          std::string metadata_key = t->key;
          std::string metadata_value = t->value;
          // CameraCalibrationType::CameraCalibrationTypeDefault = -1
          if (camera_calibration_struct_->type == CameraCalibrationType::CameraCalibrationTypeDefault) {
            camera_calibration_struct_->type = CameraCalibrationType::CameraCalibrationTypeKinect; // = 0;
            camera_calibration_struct_->extra_data.resize(2);
          }
          if (metadata_key == "K4A_COLOR_MODE") {
            camera_calibration_struct_->extra_data[1] =
                GetKinectColorResolution(metadata_value);
          } else if (metadata_key == "K4A_DEPTH_MODE") {
            camera_calibration_struct_->extra_data[0] =
                GetKinectDepthMode(metadata_value);
          }
        }

        av_codec_contexts_[i] = codec_context;
        frame_struct_buffer_ = nullptr;

        fps_ = av_q2d(av_format_context_
                          ->streams[video_stream_indexes_.begin().operator*()]
                          ->r_frame_rate);

        void *sEPointer = codec_parameter->extradata;
        size_t sESize = codec_parameter->extradata_size;
        size_t sSize = sizeof(*codec_parameter);

        std::vector<unsigned char> e(sSize);
        std::vector<unsigned char> ed(sESize);

        memcpy(&e[0], codec_parameter, sSize);
        memcpy(&ed[0], sEPointer, sESize);

        CodecParamsStruct codec_params_struct(
          //0
          CodecParamsType::CodecParamsTypeAv, e, ed);
        codec_params_structs_[i] = codec_params_struct;

        // avcodec_parameters_free(&pCodecParameter);
      }
    }
  }

  packet_ = av_packet_alloc();
  if (!packet_) {
    spdlog::error("Failed to allocated memory for AVPacket.");
    exit(-1);
  }

  frame_struct_template_.message_type = SSPMessageType::MessageTypeDefault; // = 0;
  frame_struct_template_.frame_data_type = FrameDataType::FrameDataTypeLibavPackets ; // = 1;
  frame_struct_template_.stream_id = RandomString(16);
  frame_struct_template_.device_id = 0;
  frame_struct_template_.camera_calibration_data = *camera_calibration_struct_;

  libav_ready_ = true;
}

unsigned int VideoFileReader::GetCurrentFrameId() {
  return current_frame_counter_;
}

void VideoFileReader::NextFrame() {
  if (!libav_ready_)
    Init(this->filename_);

  if (frame_struct_buffer_ != nullptr) {
    frame_structs_.clear();
    frame_structs_.push_back(frame_struct_buffer_);
    frame_struct_buffer_ = nullptr;
  }

  int error = 0;
  while (error >= 0) {
    error = av_read_frame(av_format_context_, packet_);
    // if it's the video stream
    std::vector<unsigned int>::iterator it;

    it = find(video_stream_indexes_.begin(), video_stream_indexes_.end(),
              packet_->stream_index);
    if (it != video_stream_indexes_.end()) {

      std::shared_ptr<FrameStruct> frame_struct =
          std::shared_ptr<FrameStruct>(new FrameStruct(frame_struct_template_));

      frame_struct->frame = std::vector<unsigned char>(
          &packet_->data[0], &packet_->data[0] + packet_->size);
      frame_struct->frame_id = current_frame_counter_;
      frame_struct->sensor_id = packet_->stream_index;
      frame_struct->frame_type = FrameType(packet_->stream_index);
      frame_struct->timestamps.push_back(packet_->pts);
      frame_struct->timestamps.push_back(CurrentTimeNs());
      frame_struct->codec_data = codec_params_structs_[packet_->stream_index];

      if (frame_structs_.empty() ||
          (std::abs((long)(packet_->pts -
                           (long)frame_structs_.front()->timestamps.front())) <
           10000)) {
        frame_structs_.push_back(frame_struct);
      } else if (frame_struct_buffer_ == nullptr) {
        frame_struct_buffer_ = frame_struct;
        current_frame_counter_ += 1;
        frame_struct_buffer_->frame_id = current_frame_counter_;
        av_packet_unref(packet_);
        break;
      }
    }
    av_packet_unref(packet_);
    if (error == AVERROR_EOF) {
      eof_reached_ = true;
      frame_structs_.clear();
      if (frame_struct_buffer_ != nullptr) {
        frame_struct_buffer_ = nullptr;
      }
      error = 1;
      break;
    }
  }
}

bool VideoFileReader::HasNextFrame() {
  if (!libav_ready_)
    Init(this->filename_);
  return !eof_reached_;
}

void VideoFileReader::GoToFrame(unsigned int frame_id) {
  current_frame_counter_ = frame_id;
  eof_reached_ = false;
  int error =
      av_seek_frame(av_format_context_, -1, frame_id, AVSEEK_FLAG_FRAME);
  if (error < 0) {
    spdlog::error("Error seeking to frame {}: {}", frame_id, _av_err2str(error));
  }
}

void VideoFileReader::Reset() {
  current_frame_counter_ = 0;
  eof_reached_ = false;

  int error = av_seek_frame(av_format_context_, -1, 0, AVSEEK_FLAG_BACKWARD);

  if (error < 0) {
    spdlog::error("Error seeking to frame {}: {}", 0, _av_err2str(error));
  }
}

unsigned int VideoFileReader::GetFps() {
  if (!libav_ready_)
    Init(this->filename_);
  return fps_;
}

std::vector<std::shared_ptr<FrameStruct>> VideoFileReader::GetCurrentFrame() {
  return frame_structs_;
}

typedef enum {
  VIDEO_READER_K4A_DEPTH_MODE_OFF =
      0, /**< Depth sensor will be turned off with this setting. */
  VIDEO_READER_K4A_DEPTH_MODE_NFOV_2X2BINNED, /**< Depth captured at 320x288.
                                                 Passive IR is also captured at
                                                 320x288. */
  VIDEO_READER_K4A_DEPTH_MODE_NFOV_UNBINNED,  /**< Depth captured at 640x576.
                                                 Passive IR is also captured at
                                                 640x576. */
  VIDEO_READER_K4A_DEPTH_MODE_WFOV_2X2BINNED, /**< Depth captured at 512x512.
                                                 Passive IR is also captured at
                                                 512x512. */
  VIDEO_READER_K4A_DEPTH_MODE_WFOV_UNBINNED,  /**< Depth captured at 1024x1024.
                                                 Passive IR is also captured at
                                                 1024x1024. */
  VIDEO_READER_K4A_DEPTH_MODE_PASSIVE_IR,     /**< Passive IR only, captured at
                                                 1024x1024. */
} video_reader_k4a_depth_mode_t;

typedef enum {
  VIDEO_READER_K4A_COLOR_RESOLUTION_OFF =
      0, /**< Color camera will be turned off with this setting */
  VIDEO_READER_K4A_COLOR_RESOLUTION_720P,  /**< 1280 * 720  16:9 */
  VIDEO_READER_K4A_COLOR_RESOLUTION_1080P, /**< 1920 * 1080 16:9 */
  VIDEO_READER_K4A_COLOR_RESOLUTION_1440P, /**< 2560 * 1440 16:9 */
  VIDEO_READER_K4A_COLOR_RESOLUTION_1536P, /**< 2048 * 1536 4:3  */
  VIDEO_READER_K4A_COLOR_RESOLUTION_2160P, /**< 3840 * 2160 16:9 */
  VIDEO_READER_K4A_COLOR_RESOLUTION_3072P, /**< 4096 * 3072 4:3  */
} video_reader_k4a_color_resolution_t;

std::vector<FrameType> VideoFileReader::GetType() {
  if (!libav_ready_)
    Init(this->filename_);
  std::vector<FrameType> ft;
  for (auto &t : video_stream_indexes_) ft.push_back(FrameType(t));
  return ft;
}
ushort VideoFileReader::GetKinectColorResolution(std::string &metadata_value) {
  std::string ending = metadata_value.substr(metadata_value.size() - 5, 5);
  if (ending == "_720P")
    return VIDEO_READER_K4A_COLOR_RESOLUTION_720P;
  if (ending == "1080P")
    return VIDEO_READER_K4A_COLOR_RESOLUTION_1080P;
  if (ending == "1440P")
    return VIDEO_READER_K4A_COLOR_RESOLUTION_1440P;
  if (ending == "1536P")
    return VIDEO_READER_K4A_COLOR_RESOLUTION_1536P;
  if (ending == "2160P")
    return VIDEO_READER_K4A_COLOR_RESOLUTION_2160P;
  if (ending == "3072P")
    return VIDEO_READER_K4A_COLOR_RESOLUTION_3072P;
  return 0;
}

ushort VideoFileReader::GetKinectDepthMode(std::string &metadata_value) {
  if (metadata_value == "NFOV_2X2BINNED")
    return VIDEO_READER_K4A_DEPTH_MODE_NFOV_2X2BINNED;
  if (metadata_value == "NFOV_UNBINNED")
    return VIDEO_READER_K4A_DEPTH_MODE_NFOV_UNBINNED;
  if (metadata_value == "WFOV_2X2BINNED")
    return VIDEO_READER_K4A_DEPTH_MODE_WFOV_2X2BINNED;
  if (metadata_value == "WFOV_UNBINNED")
    return VIDEO_READER_K4A_DEPTH_MODE_WFOV_UNBINNED;
  if (metadata_value == "PASSIVE_IR")
    return VIDEO_READER_K4A_DEPTH_MODE_PASSIVE_IR;
  return 0;
}

} //  namespace moetsi::ssp
