/**
 * Namespace video_utils.cc @brief Video utilities 
 */
#include "video_utils.h"
#include "libav_types.h"

namespace moetsi::ssp {

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

  // PNG decoding requires clearing frame
  if (frame->format == 2) {
    memset(frame->data[0], 0, frame->width * frame->height * 3);
  }
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
  // PNG decoding requires clearing frame
  if (frame->format == AV_PIX_FMT_GRAY16BE)
    memset(frame->data[0], 0, frame->height * frame->width * 2);
}

AVCodecParameters *getParams(FrameStruct &frame_struct) {
  if (frame_struct.codec_data.type != CodecParamsType::CodecParamsTypeAv) // 0)
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

} // namespace moetsi::ssp
