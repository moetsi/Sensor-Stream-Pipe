//
// Created by amourao on 26-06-2019.
//

#include "FrameEncoder.h"

FrameEncoder::FrameEncoder(std::string codec_parameters_file, uint _fps) {
  codec_parameters = YAML::LoadFile(codec_parameters_file);
  fps = _fps;
  av_register_all();

  ready = false;
  cParamsStruct = NULL;
  totalCurrentFrameCounter = 0;
}

FrameEncoder::FrameEncoder(YAML::Node &_codec_parameters, uint _fps) {
  codec_parameters = _codec_parameters;
  fps = _fps;
  av_register_all();

  ready = false;
  cParamsStruct = NULL;
  totalCurrentFrameCounter = 0;
}

std::vector<unsigned char> FrameEncoder::currentFrameBytes() {
  return std::vector<unsigned char>(pBuffer.front()->data, pBuffer.front()->data + pBuffer.front()->size);
}

FrameEncoder::~FrameEncoder() {
  av_packet_free(&pPacket);
  av_frame_free(&pFrame);
  avcodec_free_context(&pCodecContextEncoder);
}

void FrameEncoder::nextPacket() {
  if (!buffer.empty()) {
    FrameStruct *f = buffer.front();
    buffer.pop();
    f->frame.clear();
    delete f;

  }
  if (!pBuffer.empty()) {
    AVPacket *p = pBuffer.front();
    pBuffer.pop();
    delete[] p->data;
    delete p;

  }
}

void FrameEncoder::prepareFrame() {

  if (buffer.front()->frameDataType == 2) {
    uint8_t *inData[1] = {&buffer.front()->frame[8]};
    int inLinesize[1] = {4 * pFrame->width};

    sws_scale(sws_ctx, (const uint8_t *const *) inData, inLinesize,
              0, pFrame->height, pFrame->data, pFrame->linesize);

  } else if (buffer.front()->frameDataType == 3) {
    if (pFrame->format == AV_PIX_FMT_GRAY12LE) {
      // TODO: replace with straight mem copy
      int i = 0;
      uint8_t *data = &buffer.front()->frame[8];
      memcpy(pFrame->data[0], data, pFrame->height * pFrame->width);
      /*
      for (uint y = 0; y < pFrame->height; y++) {
        for (uint x = 0; x < pFrame->width; x++) {
          uint lower = data[i];
          uint upper = data[i + 1];

          pFrame->data[0][i] = lower;
          i++;
          pFrame->data[0][i] = upper;
          i++;
        }
      }
       */
    } else if (pFrame->format == AV_PIX_FMT_GRAY16BE) { // PNG GRAY16 TO gray12le
      int i = 0;
      uint8_t *data = &buffer.front()->frame[8];
      for (uint y = 0; y < pFrame->height; y++) {
        for (uint x = 0; x < pFrame->width; x++) {
          pFrame->data[0][i] = data[i + 1];
          pFrame->data[0][i + 1] = data[i];
          i += 2;
        }
      }
    } else {
      int i = 0;
      uint8_t *data = &buffer.front()->frame[8];
      float coeff = (float) MAX_DEPTH_VALUE_8_BITS / MAX_DEPTH_VALUE_12_BITS;
      for (uint y = 0; y < pFrame->height; y++) {
        for (uint x = 0; x < pFrame->width; x++) {
          uint lower = data[i * 2];
          uint upper = data[i * 2 + 1];
          ushort value = upper << 8 | lower;

          pFrame->data[0][i++] = std::min((uint) (value * coeff), (uint) 255);
        }
      }
      for (uint y = 0; y < pFrame->height / 2; y++) {
        for (uint x = 0; x < pFrame->width / 2; x++) {
          pFrame->data[1][y * pFrame->linesize[1] + x] = 128;
          pFrame->data[2][y * pFrame->linesize[2] + x] = 128;
        }
      }
    }

  } else if (buffer.front()->frameDataType == 0) {

    AVFrame *pFrameO = av_frame_alloc();

    std::vector<unsigned char> frameData = buffer.front()->frame;

    id.imageBufferToAVFrame(frameData, pFrameO);

    if (pFrameO->format == pFrame->format) {
      av_frame_copy(pFrame, pFrameO);

    } else if (pFrameO->format == AV_PIX_FMT_GRAY16BE &&
               pFrame->format == AV_PIX_FMT_GRAY12LE) { // PNG GRAY16 TO gray12le
      int i = 0;
      for (uint y = 0; y < pFrameO->height; y++) {
        for (uint x = 0; x < pFrameO->width; x++) {
          uint lower = pFrameO->data[0][y * pFrameO->linesize[0] + x * 2];
          uint upper = pFrameO->data[0][y * pFrameO->linesize[0] + x * 2 + 1];

          pFrame->data[0][i++] = upper;
          pFrame->data[0][i++] = lower;
        }
      }
    } else if (pFrameO->format == AV_PIX_FMT_GRAY16BE &&
               (pFrame->format == AV_PIX_FMT_YUV420P ||
                pFrame->format == AV_PIX_FMT_YUV422P ||
                pFrame->format == AV_PIX_FMT_YUV444P)) { // PNG GRAY16 TO YUV
      // TODO: remove redundant call
      sws_scale(sws_ctx, (const uint8_t *const *) pFrameO->data, pFrameO->linesize,
                0, pFrameO->height, pFrame->data, pFrame->linesize);

      int i = 0;
      float coeff = (float) MAX_DEPTH_VALUE_8_BITS / MAX_DEPTH_VALUE_12_BITS;
      // TODO: replace with straight mem copy
      for (uint y = 0; y < pFrameO->height; y++) {
        for (uint x = 0; x < pFrameO->width; x++) {
          uint lower = pFrameO->data[0][y * pFrameO->linesize[0] + x * 2];
          uint upper = pFrameO->data[0][y * pFrameO->linesize[0] + x * 2 + 1];
          ushort value = lower << 8 | upper;

          pFrame->data[0][i++] = std::min((ushort) (value * coeff), (ushort) 255);
        }
      }

      // TODO: encode the missing bits in the other channels (see the research
      // papers)
      /*
      for (uint y = 0; y < pFrame->height; y++) {
          for (uint x = 0; x < pFrame->width; x++) {
              pFrame->data[1][y * pFrame->linesize[1] + x] = 128;
              pFrame->data[2][y * pFrame->linesize[2] + x] = 128;
          }
      }*/
    } else {
      // YUV to YUV
      sws_scale(sws_ctx, (const uint8_t *const *) pFrameO->data, pFrameO->linesize,
                0, pFrameO->height, pFrame->data, pFrame->linesize);
    }

    av_frame_free(&pFrameO);
  }
  /*
  cv::Mat frameOri = cv::imdecode(frameData, CV_LOAD_IMAGE_UNCHANGED);

  int ret = av_frame_make_writable(pFrame);
  if (ret < 0) {
      std::cerr << "Error making frames writable" << std::endl;
      exit(1);
  }
  // yuv420p 640 320 320 format 0
  // yuv422p 640 320 320 format 4


   */
}

void FrameEncoder::encodeA(AVCodecContext *enc_ctx, AVFrame *frame,
                           AVPacket *pkt) {
  int ret;
  /* send the frame to the encoder */
  ret = avcodec_send_frame(enc_ctx, frame);
  if (ret < 0) {
    fprintf(stderr, "Error sending a frame for encoding\n");
    exit(1);
  }

    ret = avcodec_receive_packet(enc_ctx, pkt);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
      return;
    else if (ret < 0) {
      fprintf(stderr, "Error during encoding\n");
      exit(1);
    }

  AVPacket *newPacket = new AVPacket(*pPacket);
  pBuffer.push(newPacket);
  pBuffer.back()->data = reinterpret_cast<uint8_t *>(new uint64_t[
  (pPacket->size + FF_INPUT_BUFFER_PADDING_SIZE) / sizeof(uint64_t) + 1]);
  memcpy(pBuffer.back()->data, pPacket->data, pPacket->size);



}

void FrameEncoder::encode() {

  int ret;

  int i = 0;

  prepareFrame();

  pFrame->pts = totalCurrentFrameCounter++;

  encodeA(pCodecContextEncoder, pFrame, pPacket);
}

void FrameEncoder::init(FrameStruct *fs) {
  int ret;

  std::cout << codec_parameters << std::endl;

  pCodecEncoder = avcodec_find_encoder_by_name(
          codec_parameters["codec_name"].as<std::string>().c_str());
  pCodecContextEncoder = avcodec_alloc_context3(pCodecEncoder);
  pPacket = av_packet_alloc();
  pCodecParametersEncoder = avcodec_parameters_alloc();

  int width, height, pxl_format;

  if (fs->frameDataType == 0 || fs->frameDataType == 1) {
    AVFrame *pFrameO = av_frame_alloc();

    id.imageBufferToAVFrame(fs->frame, pFrameO);

    width = pFrameO->width;
    height = pFrameO->height;
    pxl_format = pFrameO->format;

    av_frame_free(&pFrameO);
  } else if (fs->frameDataType == 2) {
    memcpy(&width, &fs->frame[0], sizeof(int));
    memcpy(&height, &fs->frame[4], sizeof(int));
    pxl_format = AV_PIX_FMT_BGRA;
  } else if (fs->frameDataType == 3) {
    memcpy(&width, &fs->frame[0], sizeof(int));
    memcpy(&height, &fs->frame[4], sizeof(int));
    pxl_format = AV_PIX_FMT_GRAY16LE;
  }


  pCodecContextEncoder->width = width;
  pCodecContextEncoder->height = height;
  /* frames per second */
  pCodecContextEncoder->time_base = (AVRational) {1, (int) fps};
  pCodecContextEncoder->framerate = (AVRational) {(int) fps, 1};
  pCodecContextEncoder->gop_size = 0;

  pCodecContextEncoder->bit_rate_tolerance = 0;
  pCodecContextEncoder->rc_max_rate = 0;
  pCodecContextEncoder->rc_buffer_size = 0;
  pCodecContextEncoder->max_b_frames = 0;
  pCodecContextEncoder->delay = 0;

  /* emit one intra frame every ten frames
   * check frame pict_type before passing frame
   * to encoder, if frame->pict_type is AV_PICTURE_TYPE_I
   * then gop_size is ignored and the output of encoder
   * will always be I frame irrespective to gop_size
   */

  // TDOO: check delay:
  // https://ffmpeg.org/pipermail/libav-user/2014-December/007672.html
  // 1
  pCodecContextEncoder->bit_rate = codec_parameters["bit_rate"].as<int>(); // 1

  // fmpeg -h encoder=hevc
  // libx264:                 yuv420p yuvj420p yuv422p yuvj422p yuv444p yuvj444p
  // nv12 nv16 nv21 libx264rgb:              bgr0 bgr24 rgb24 libx265: yuv420p
  // yuv422p yuv444p gbrp yuv420p10le yuv422p10le yuv444p10le gbrp10le
  // yuv420p12le yuv422p12le yuv444p12le gbrp12le gray gray10le gray12le
  // hevc_nvenc (gpu x265):   yuv420p nv12 p010le yuv444p yuv444p16le bgr0 rgb0
  // cuda nv12 p010le nvenc_h264 (gpu x264):   yuv420p nv12 p010le yuv444p
  // yuv444p16le bgr0 rgb0 cuda

  pCodecContextEncoder->pix_fmt =
          av_get_pix_fmt(codec_parameters["pix_fmt"].as<std::string>().c_str());

  YAML::Node codec_parameters_options = codec_parameters["options"];

  for (YAML::const_iterator it = codec_parameters_options.begin();
       it != codec_parameters_options.end(); ++it) {
    av_opt_set(pCodecContextEncoder->priv_data,
               it->first.as<std::string>().c_str(),
               it->second.as<std::string>().c_str(), AV_OPT_SEARCH_CHILDREN);
  }

  av_opt_set(pCodecContextEncoder->priv_data, "tune", "delay", 0);
  av_opt_set(pCodecContextEncoder->priv_data, "tune", "zerolatency", 1);
  av_opt_set(pCodecContextEncoder->priv_data, "rcParams", "zeroReorderDelay",
             1);


  pCodecParametersEncoder->codec_type = AVMEDIA_TYPE_VIDEO;

  pCodecParametersEncoder->codec_id = pCodecEncoder->id;
  pCodecParametersEncoder->codec_tag = pCodecContextEncoder->codec_tag;
  pCodecParametersEncoder->bit_rate = pCodecContextEncoder->bit_rate;
  pCodecParametersEncoder->bits_per_coded_sample =
          pCodecContextEncoder->bits_per_coded_sample;
  pCodecParametersEncoder->bits_per_raw_sample =
          pCodecContextEncoder->bits_per_raw_sample;
  pCodecParametersEncoder->profile = pCodecContextEncoder->level;
  pCodecParametersEncoder->width = pCodecContextEncoder->width;
  pCodecParametersEncoder->height = pCodecContextEncoder->height;

  pCodecParametersEncoder->color_space = pCodecContextEncoder->colorspace;
  pCodecParametersEncoder->sample_rate = pCodecContextEncoder->sample_rate;

  ret = avcodec_open2(pCodecContextEncoder, pCodecEncoder, NULL);
  if (ret < 0) {
    std::cerr << "Could not open codec: " << av_err2str(ret) << std::endl;
    exit(1);
  }

  pFrame = av_frame_alloc();
  if (!pFrame) {
    std::cerr << "Could not allocate video frame." << std::endl;
    exit(1);
  }

  pFrame->format = pCodecContextEncoder->pix_fmt;
  pFrame->width = pCodecContextEncoder->width;
  pFrame->height = pCodecContextEncoder->height;

  pFrame->pts = 0;
  pFrame->pkt_dts = 0;

  ret = av_frame_get_buffer(pFrame, 30);
  if (ret < 0) {
    std::cerr << "Could not allocate the video frame data" << std::endl;
    exit(1);
  }

  sws_ctx = sws_getContext(width, height,
                           (AVPixelFormat) pxl_format, pFrame->width,
                           pFrame->height, (AVPixelFormat) pFrame->format,
                           SWS_BILINEAR, NULL, NULL, NULL);


}

CodecParamsStruct *FrameEncoder::getCodecParamsStruct() {

  if (cParamsStruct == NULL) {

    void *sEPointer = pCodecParametersEncoder->extradata;
    size_t sESize = pCodecParametersEncoder->extradata_size;
    size_t sSize = sizeof(*pCodecParametersEncoder);

    std::vector<unsigned char> e(sSize);
    std::vector<unsigned char> ed(sESize);

    memcpy(&e[0], pCodecParametersEncoder, sSize);
    memcpy(&ed[0], sEPointer, sESize);
    cParamsStruct = new CodecParamsStruct(e, ed);

  }

  return cParamsStruct;
}

unsigned int FrameEncoder::currentFrameId() { return totalCurrentFrameCounter; }

FrameStruct *FrameEncoder::currentFrame() {
  FrameStruct *f = new FrameStruct(*buffer.front());

  f->messageType = 0;
  f->frameDataType = 1;
  f->codec_data = *getCodecParamsStruct();
  f->frame = currentFrameBytes();
  f->frameId = totalCurrentFrameCounter;

  return f;
}

FrameStruct *FrameEncoder::currentFrameOriginal() {
  if (buffer.empty())
    return NULL;
  return buffer.front();
}

void FrameEncoder::addFrameStruct(FrameStruct *fs) {
  if (!ready) {
    ready = true;
    init(fs);
  }

  buffer.push(fs);
  encode();
}

uint FrameEncoder::getFps() { return fps; }

bool FrameEncoder::hasNextPacket() { return !pBuffer.empty(); }
