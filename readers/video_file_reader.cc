//
// Created by amourao on 26-06-2019.
//

#include "video_file_reader.h"

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

  for (auto fs : frame_structs_)
    delete fs;
  if (frame_struct_buffer_ != nullptr)
    delete frame_struct_buffer_;
}

void VideoFileReader::Init(std::string &filename) {
  av_register_all();
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

    AVCodec *codec = avcodec_find_decoder(codec_parameter->codec_id);
    if (codec == NULL) {
      spdlog::warn("Non video stream detected ({}), skipping", i);
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

        CodecParamsStruct codec_params_struct(0, e, ed);
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

  frame_struct_template_.message_type = 0;

  frame_struct_template_.frame_data_type = 1;
  frame_struct_template_.stream_id = RandomString(16);
  frame_struct_template_.device_id = 0;

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

      FrameStruct *frameStruct = new FrameStruct(frame_struct_template_);
      frameStruct->frame = std::vector<unsigned char>(
          &packet_->data[0], &packet_->data[0] + packet_->size);
      frameStruct->frame_id = current_frame_counter_;
      frameStruct->sensor_id = packet_->stream_index;
      frameStruct->frame_type = packet_->stream_index;
      frameStruct->timestamps.push_back(packet_->pts);
      frameStruct->timestamps.push_back(CurrentTimeMs());
      frameStruct->codec_data = codec_params_structs_[packet_->stream_index];

      if (frame_structs_.empty() ||
          (std::abs((long)(packet_->pts -
                           (long)frame_structs_.front()->timestamps.front())) <
           10000)) {
        frame_structs_.push_back(frameStruct);
      } else if (frame_struct_buffer_ == nullptr) {
        frame_struct_buffer_ = frameStruct;
        current_frame_counter_ += 1;
        frame_struct_buffer_->frame_id = current_frame_counter_;
        av_packet_unref(packet_);
        break;
      }
    }
    av_packet_unref(packet_);
    if (error == AVERROR_EOF) {
      eof_reached_ = true;
      for (auto fs : frame_structs_)
        delete fs;
      frame_structs_.clear();
      if (frame_struct_buffer_ != nullptr) {
        delete frame_struct_buffer_;
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
  int error = av_seek_frame(av_format_context_, -1, frame_id, AVSEEK_FLAG_FRAME);
  if (error < 0) {
    spdlog::error("Error seeking to frame {}: {}", frame_id, av_err2str(error));
  }
}

void VideoFileReader::Reset() {
  current_frame_counter_ = 0;
  eof_reached_ = false;

  int error = av_seek_frame(av_format_context_, -1, 0, AVSEEK_FLAG_BACKWARD);

  if (error < 0) {
    spdlog::error("Error seeking to frame {}: {}", 0, av_err2str(error));
  }
}

unsigned int VideoFileReader::GetFps() {
  if (!libav_ready_)
    Init(this->filename_);
  return fps_;
}

std::vector<FrameStruct *> VideoFileReader::GetCurrentFrame() {
  return frame_structs_;
}

std::vector<unsigned int> VideoFileReader::GetType() {
  if (!libav_ready_)
    Init(this->filename_);
  return video_stream_indexes_;
}