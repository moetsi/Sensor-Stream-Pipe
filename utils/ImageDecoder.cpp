//
// Created by amourao on 28/08/19.
//

#include "ImageDecoder.h"

int ImageDecoder::decode_packet(AVFrame *pFrame) {
  // Supply raw packet data as input to a decoder
  // https://ffmpeg.org/doxygen/trunk/group__lavc__decoding.html#ga58bc4bf1e0ac59e27362597e467efff3
  int response = avcodec_send_packet(pCodecContext, pPacket);

  if (response < 0) {
    std::cout << "Error while sending a packet to the decoder: "
              << av_err2str(response) << std::endl;
    return response;
  }

  while (response >= 0) {
    // Return decoded output data (into a frame) from a decoder
    // https://ffmpeg.org/doxygen/trunk/group__lavc__decoding.html#ga11e6542c4e66d3028668788a1a74217c
    response = avcodec_receive_frame(pCodecContext, pFrame);
    if (response == AVERROR(EAGAIN) || response == AVERROR_EOF) {
      break;
    } else if (response < 0) {
      std::cout << "Error while receiving a frame from the decoder: "
                << av_err2str(response) << std::endl;
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
  libAVReady = false;
  cParamsStruct = NULL;
  av_register_all();
}

ImageDecoder::~ImageDecoder() {}

void ImageDecoder::init(std::vector<unsigned char> &buffer) {

  struct buffer_data bd = {0};
  bd.ptr = (uint8_t *)&buffer[0];
  bd.size = buffer.size();

  int ret = 0;

  if (!(pFormatContext = avformat_alloc_context())) {
    ret = AVERROR(ENOMEM);
  }

  avio_ctx_buffer = (unsigned char *)av_malloc(avio_ctx_buffer_size);
  if (!avio_ctx_buffer) {
    ret = AVERROR(ENOMEM);
    exit(1);
  }
  avio_ctx = avio_alloc_context(avio_ctx_buffer, avio_ctx_buffer_size, 0, &bd,
                                &read_packet, NULL, NULL);
  if (!avio_ctx) {
    ret = AVERROR(ENOMEM);
    exit(1);
  }
  pFormatContext->pb = avio_ctx;

  ret = avformat_open_input(&pFormatContext, NULL, NULL, NULL);
  if (ret < 0) {
    fprintf(stderr, "Could not open input\n");
    exit(1);
  }

  ret = avformat_find_stream_info(pFormatContext, NULL);
  if (ret < 0) {
    fprintf(stderr, "Could not find stream information\n");
    exit(1);
  }

  // av_dump_format(pFormatContext, 0, "nofile", 0);

  pCodecParameters = NULL;

  // loop though all the streams and print its main information
  for (int i = 0; i < pFormatContext->nb_streams; i++) {
    pCodecParameters = pFormatContext->streams[i]->codecpar;
    pCodec = avcodec_find_decoder(pCodecParameters->codec_id);

    if (pCodec != NULL && pCodecParameters->codec_type == AVMEDIA_TYPE_VIDEO) {
      break;
    }
  }

  pCodecContext = avcodec_alloc_context3(pCodec);
  if (!pCodecContext) {
    std::cout << "failed to allocated memory for AVCodecContext" << std::endl;
    exit(-1);
  }

  if (avcodec_parameters_to_context(pCodecContext, pCodecParameters) < 0) {
    std::cout << "failed to copy codec params to codec context" << std::endl;
    exit(-1);
  }

  if (avcodec_open2(pCodecContext, pCodec, NULL) < 0) {
    std::cout << "failed to open codec through avcodec_open2" << std::endl;
    exit(-1);
  }

  pPacket = av_packet_alloc();
  if (!pPacket) {
    std::cout << "failed to allocated memory for AVPacket" << std::endl;
    exit(-1);
  }

  libAVReady = true;
}

CodecParamsStruct *ImageDecoder::getCodecParamsStruct() {

  if (cParamsStruct == NULL) {

    void *sEPointer = pCodecParameters->extradata;
    size_t sESize = pCodecParameters->extradata_size;
    size_t sSize = sizeof(*pCodecParameters);

    std::vector<unsigned char> e(sSize);
    std::vector<unsigned char> ed(sESize);

    memcpy(&e[0], pCodecParameters, sSize);
    memcpy(&ed[0], sEPointer, sESize);
    cParamsStruct = new CodecParamsStruct(0, e, ed);
  }

  return cParamsStruct;
}

// http://guru-coder.blogspot.com/2014/01/in-memory-jpeg-decode-using-ffmpeg.html
void ImageDecoder::imageBufferToAVFrame(FrameStruct *fs, AVFrame *pFrame) {

  std::vector<unsigned char> buffer = fs->frame;

  // TODO: do not create a decoder for each single frame
  init(buffer);

  fs->codec_data = *getCodecParamsStruct();

  int response = 0;

  av_read_frame(pFormatContext, pPacket);
  response = decode_packet(pFrame);


  av_packet_free(&pPacket);
  avcodec_free_context(&pCodecContext);
  // avformat_free_context(pFormatContext);

  avformat_close_input(&pFormatContext);
  if (avio_ctx) {
    av_freep(&avio_ctx->buffer);
    av_freep(&avio_ctx);
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