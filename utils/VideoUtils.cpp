#include "VideoUtils.h"

void avframeToMatYUV(const AVFrame *frame, cv::Mat &image) {
  int width = frame->width;
  int height = frame->height;

  SwsContext *conversion;

  image = cv::Mat(height, width, CV_8UC3);
  conversion =
      sws_getContext(width, height, (AVPixelFormat)frame->format, width, height,
                     AV_PIX_FMT_BGR24, SWS_FAST_BILINEAR, NULL, NULL, NULL);
  int cvLinesizes[1];
  cvLinesizes[0] = image.step1();

  sws_scale(conversion, frame->data, frame->linesize, 0, height, &image.data,
            cvLinesizes);
  sws_freeContext(conversion);
}

void avframeToMatGray(const AVFrame *frame, cv::Mat &image) {
  int width = frame->width;
  int height = frame->height;

  image = cv::Mat(height, width, CV_16UC1);
  for (uint y = 0; y < frame->height; y++) {
    for (uint x = 0; x < frame->width; x++) {
      ushort lower = frame->data[0][y * frame->linesize[0] + x * 2];
      ushort upper = frame->data[0][y * frame->linesize[0] + x * 2 + 1];
      ushort value;

      if (frame->format == AV_PIX_FMT_GRAY12LE) {
        value = upper << 8 | lower;
        image.at<ushort>(y, x) = value;
      } else if (frame->format == AV_PIX_FMT_GRAY16BE) {
        value = lower << 8 | upper;
        image.at<ushort>(y, x) = value;
      }
    }
  }
  if (frame->format == AV_PIX_FMT_GRAY16BE)
    memset(frame->data[0], 0, frame->height * frame->width * 2);
}

void prepareDecodingStruct(
        FrameStruct *f, std::unordered_map<std::string, AVCodec *> &pCodecs,
        std::unordered_map<std::string, AVCodecContext *> &pCodecContexts,
        std::unordered_map<std::string, AVCodecParameters *> &pCodecParameters) {
  AVCodecParameters *pCodecParameter = f->codec_data.getParams();
  AVCodec *pCodec = avcodec_find_decoder(f->codec_data.getParams()->codec_id);
  AVCodecContext *pCodecContext = avcodec_alloc_context3(pCodec);

  if (!pCodecContext) {
    std::cerr << "failed to allocated memory for AVCodecContext" << std::endl;
    exit(1);
  }

  if (avcodec_parameters_to_context(pCodecContext, pCodecParameter) < 0) {
    std::cerr << ("failed to copy codec params to codec context") << std::endl;
    exit(1);
  }

  if (avcodec_open2(pCodecContext, pCodec, NULL) < 0) {
    std::cerr << ("failed to open codec through avcodec_open2") << std::endl;
    exit(1);
  }

  pCodecs[f->streamId + std::to_string(f->sensorId)] = pCodec;
  pCodecContexts[f->streamId + std::to_string(f->sensorId)] = pCodecContext;
  pCodecParameters[f->streamId + std::to_string(f->sensorId)] = pCodecParameter;
}
