//
// Created by amourao on 28/08/19.
//

#include "image_decoder.h"

int ImageDecoder::DecodePacket(AVFrameSharedP pFrame) {
  // Supply raw packet data as input to a decoder
  // https://ffmpeg.org/doxygen/trunk/group__lavc__decoding.html#ga58bc4bf1e0ac59e27362597e467efff3
  int response = avcodec_send_packet(av_codec_context_, packet_);

  if (response < 0) {
    spdlog::error("Error while sending a packet to the decoder: {}",
                  av_err2str(response));
    return response;
  }

  while (response >= 0) {
    // Return decoded output data (into a frame) from a decoder
    // https://ffmpeg.org/doxygen/trunk/group__lavc__decoding.html#ga11e6542c4e66d3028668788a1a74217c
    response = avcodec_receive_frame(av_codec_context_, pFrame.get());
    if (response == AVERROR(EAGAIN) || response == AVERROR_EOF) {
      break;
    } else if (response < 0) {
      spdlog::error("Error while receiving a frame from the decoder: {}",
                    av_err2str(response));
      return response;
    }

    if (response >= 0) {
      break;
    }
  }
  return 0;
}

struct buffer_data {
  uint8_t *ptr;
  size_t size; ///< size left in the buffer
};

static int read_packet(void *opaque, uint8_t *buf, int buf_size) {
  struct buffer_data *bd = (struct buffer_data *)opaque;
  buf_size = FFMIN(buf_size, bd->size);
  /* copy internal buffer data to buf */
  memcpy(buf, bd->ptr, buf_size);
  bd->ptr += buf_size;
  bd->size -= buf_size;

  return buf_size;
}

ImageDecoder::ImageDecoder() {
  libav_ready_ = false;
  codec_params_struct_ = NULL;
  av_format_context_ = NULL;
  avio_context_ = NULL;
  avio_ctx_buffer_ = NULL;
  av_register_all();
}

ImageDecoder::~ImageDecoder() {}

void ImageDecoder::Init(std::vector<unsigned char> &buffer) {

  struct buffer_data bd = {0, 0};
  bd.ptr = (uint8_t *)&buffer[0];
  bd.size = buffer.size();

  int ret = 0;

  if (!(av_format_context_ = avformat_alloc_context())) {
    ret = AVERROR(ENOMEM);
  }

  avio_ctx_buffer_ = (unsigned char *)av_malloc(avio_ctx_buffer_size_);
  if (!avio_ctx_buffer_) {
    ret = AVERROR(ENOMEM);
    exit(1);
  }
  avio_context_ = avio_alloc_context(avio_ctx_buffer_, avio_ctx_buffer_size_, 0,
                                     &bd, &read_packet, NULL, NULL);
  if (!avio_context_) {
    ret = AVERROR(ENOMEM);
    exit(1);
  }
  av_format_context_->pb = avio_context_;

  ret = avformat_open_input(&av_format_context_, NULL, NULL, NULL);
  if (ret < 0) {
    spdlog::error("Could not open input.");
    exit(1);
  }

  ret = avformat_find_stream_info(av_format_context_, NULL);
  if (ret < 0) {
    spdlog::error("Could not find stream information.");
    exit(1);
  }

  // av_dump_format(pFormatContext, 0, "nofile", 0);

  av_codec_parameters_ = NULL;

  // loop though all the streams and print its main information
  for (unsigned int i = 0; i < av_format_context_->nb_streams; i++) {
    av_codec_parameters_ = av_format_context_->streams[i]->codecpar;
    codec_ = avcodec_find_decoder(av_codec_parameters_->codec_id);

    if (codec_ != NULL &&
        av_codec_parameters_->codec_type == AVMEDIA_TYPE_VIDEO) {
      break;
    }
  }

  av_codec_context_ = avcodec_alloc_context3(codec_);
  if (!av_codec_context_) {
    spdlog::error("Failed to allocated memory for AVCodecContext.");
    exit(-1);
  }

  if (avcodec_parameters_to_context(av_codec_context_, av_codec_parameters_) <
      0) {
    spdlog::error("Failed to copy codec params to codec context.");
    exit(-1);
  }

  if (avcodec_open2(av_codec_context_, codec_, NULL) < 0) {
    spdlog::error("Failed to open codec through avcodec_open2.");
    exit(-1);
  }

  packet_ = av_packet_alloc();
  if (!packet_) {
    spdlog::error("Failed to allocated memory for AVPacket.");
    exit(-1);
  }

  libav_ready_ = true;
}

CodecParamsStruct *ImageDecoder::GetCodecParamsStruct() {

  if (codec_params_struct_ == NULL) {

    void *extra_data_pointer = av_codec_parameters_->extradata;
    size_t extra_data_size = av_codec_parameters_->extradata_size;
    size_t data_size = sizeof(*av_codec_parameters_);

    std::vector<unsigned char> data_buffer(data_size);
    std::vector<unsigned char> extra_data_buffer(extra_data_size);

    memcpy(&data_buffer[0], av_codec_parameters_, data_size);
    memcpy(&extra_data_buffer[0], extra_data_pointer, extra_data_size);
    codec_params_struct_ =
        new CodecParamsStruct(0, data_buffer, extra_data_buffer);
  }

  return codec_params_struct_;
}

// http://guru-coder.blogspot.com/2014/01/in-memory-jpeg-decode-using-ffmpeg.html
void ImageDecoder::ImageBufferToAVFrame(std::shared_ptr<FrameStruct> &fs,
                                        AVFrameSharedP pFrame) {

  std::vector<unsigned char> buffer = fs->frame;

  // TODO: do not create a decoder for each single frame
  Init(buffer);

  fs->codec_data = *GetCodecParamsStruct();

  av_read_frame(av_format_context_, packet_);
  DecodePacket(pFrame);

  av_packet_free(&packet_);
  avcodec_free_context(&av_codec_context_);
  // avformat_free_context(pFormatContext);

  avformat_close_input(&av_format_context_);
  if (avio_context_) {
    av_freep(&avio_context_->buffer);
    av_freep(&avio_context_);
  }
  //buffer.clear();

}


/*
int main(){

    char *input_filename = "/home/amourao/O Revisionista
Bloopers-LNiZG48b0W0.mp4";

    uint8_t *buffer = NULL, *avio_ctx_buffer = NULL;
    size_t buffer_size, avio_ctx_buffer_size = 4096;
    int ret = av_file_map(input_filename, &buffer, &buffer_size, 0, NULL);

    ImageDecoder id;

    std::vector<unsigned char> vecBuf(buffer, buffer+buffer_size);

    id.imageBufferToAVFrame(vecBuf);

    input_filename = "/home/amourao/Firefox_wallpaper.png";
    ret = av_file_map(input_filename, &buffer, &buffer_size, 0, NULL);
    vecBuf = std::vector<unsigned char>(buffer, buffer+buffer_size);

    id.imageBufferToAVFrame(vecBuf);

    av_file_unmap(buffer, buffer_size);


}
*/