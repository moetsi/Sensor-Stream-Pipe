#include "video_utils.h"


void AVFrameToMatYUV(const AVFrame *frame, cv::Mat &image) {
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

void AVFrameToMatGray(const AVFrame *frame, cv::Mat &image) {
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

bool FrameStructToMat(FrameStruct &f, cv::Mat &img,
                      std::unordered_map<std::string, IDecoder *> &decoders) {
  std::string decoder_id = f.stream_id + std::to_string(f.sensor_id);

  if (decoders.find(decoder_id) == decoders.end()) {
    CodecParamsStruct data = f.codec_data;
    if (data.type == 0) {
      LibAvDecoder *fd = new LibAvDecoder();
      fd->Init(getParams(f));
      decoders[decoder_id] = fd;
    } else if (data.type == 1) {
#ifdef SSP_WITH_NVPIPE_SUPPORT
      NvDecoder *fd = new NvDecoder();
      fd->Init(data.data);
      decoders[decoder_id] = fd;
#else
      spdlog::error("SSP compiled without \"nvenc\" reader support. Set to "
                    "SSP_WITH_NVPIPE_SUPPORT=ON when configuring with cmake");
      exit(1);
#endif
    } else if (data.type == 2) {
      ZDepthDecoder *fd = new ZDepthDecoder();
      fd->Init(data.data);
      decoders[decoder_id] = fd;
    }
  }

  bool img_changed = false;

  if (f.frame_data_type == 0) {
    img = cv::imdecode(f.frame, CV_LOAD_IMAGE_UNCHANGED);
    img_changed = true;
  } else if (f.frame_data_type == 2) {
    int rows, cols;
    memcpy(&cols, &f.frame[0], sizeof(int));
    memcpy(&rows, &f.frame[4], sizeof(int));
    img = cv::Mat(rows, cols, CV_8UC4, (void *)&f.frame[8], cv::Mat::AUTO_STEP);
    img_changed = true;
  } else if (f.frame_data_type == 3) {
    int rows, cols;
    memcpy(&cols, &f.frame[0], sizeof(int));
    memcpy(&rows, &f.frame[4], sizeof(int));
    img =
        cv::Mat(rows, cols, CV_16UC1, (void *)&f.frame[8], cv::Mat::AUTO_STEP);
    img_changed = true;
  } else if (f.frame_data_type == 1) {

    IDecoder *decoder;

    if (decoders.find(decoder_id) == decoders.end()) {
      CodecParamsStruct data = f.codec_data;
      if (data.type == 0) {
        LibAvDecoder *fd = new LibAvDecoder();
        fd->Init(getParams(f));
        decoders[decoder_id] = fd;
#ifdef SSP_WITH_NVPIPE_SUPPORT
      } else if (data.type == 1) {
        NvDecoder *fd = new NvDecoder();
        fd->Init(data.data);
        decoders[decoder_id] = fd;
#endif
      }
    }

    decoder = decoders[decoder_id];

    img = decoder->Decode(&f);
    img_changed = true;

    f.frame.clear();
  }
  return img_changed;
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