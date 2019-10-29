#include "video_utils.h"
#include "libav_types.h"

void AVFrameToMatYUV(AVFrameSharedP& frame, cv::Mat &image) {
  int width = frame->width;
  int height = frame->height;

  SwsContext *conversion;

  image = cv::Mat(height, width, CV_8UC3);
  conversion =
      sws_getContext(width, height, (AVPixelFormat)frame->format, width, height,
                     AV_PIX_FMT_BGR24, SWS_FAST_BILINEAR, NULL, NULL, NULL);
  int cv_linesizes[1];
  cv_linesizes[0] = image.step1();

  sws_scale(conversion, frame->data, frame->linesize, 0, height, &image.data,
            cv_linesizes);
  sws_freeContext(conversion);
}

void AVFrameToMatGray(AVFrameSharedP& frame, cv::Mat &image) {
  int width = frame->width;
  int height = frame->height;

  image = cv::Mat(height, width, CV_16UC1);
  for (int y = 0; y < frame->height; y++) {
    for (int x = 0; x < frame->width; x++) {
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

void PrepareDecodingStruct(
    FrameStruct *f, std::unordered_map<std::string, AVCodec *> &pCodecs,
    std::unordered_map<std::string, AVCodecContext *> &pCodecContexts,
    std::unordered_map<std::string, AVCodecParameters *> &pCodecParameters) {
  AVCodecParameters *pCodecParameter = getParams(*f);
  AVCodec *pCodec = avcodec_find_decoder(pCodecParameter->codec_id);
  AVCodecContext *pCodecContext = avcodec_alloc_context3(pCodec);

  if (!pCodecContext) {
    spdlog::error("Failed to allocated memory for AVCodecContext.");
    exit(1);
  }

  if (avcodec_parameters_to_context(pCodecContext, pCodecParameter) < 0) {
    spdlog::error("Failed to copy codec params to codec context.");
    exit(1);
  }

  if (avcodec_open2(pCodecContext, pCodec, NULL) < 0) {
    spdlog::error("Failed to open codec through avcodec_open2.");
    exit(1);
  }

  pCodecs[f->stream_id + std::to_string(f->sensor_id)] = pCodec;
  pCodecContexts[f->stream_id + std::to_string(f->sensor_id)] = pCodecContext;
  pCodecParameters[f->stream_id + std::to_string(f->sensor_id)] =
      pCodecParameter;
}

AVCodecParameters *getParams(FrameStruct &frame_struct) {
  if (frame_struct.codec_data.type != 0)
    return NULL;
  AVCodecParameters *results = avcodec_parameters_alloc();
  results->extradata = NULL;
  results->extradata_size = 0;
  memcpy(results, &frame_struct.codec_data.data[0],
         frame_struct.codec_data.data.size());
  results->extradata = (uint8_t *)av_mallocz(
      frame_struct.codec_data.extra_data.size() + AV_INPUT_BUFFER_PADDING_SIZE);
  memcpy(results->extradata, &frame_struct.codec_data.extra_data[0],
         frame_struct.codec_data.extra_data.size());
  results->extradata_size = frame_struct.codec_data.extra_data.size();
  return results;
}