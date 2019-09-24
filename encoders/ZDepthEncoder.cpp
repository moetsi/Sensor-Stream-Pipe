//
// Created by amourao on 23-09-2019.
//

#include "ZDepthEncoder.h"

ZDepthEncoder::ZDepthEncoder(int _fps) {
  fps = _fps;
  totalCurrentFrameCounter = 0;
  frameCompressed = nullptr;
  frameOriginal = nullptr;
  paramsStruct = nullptr;
  fd = nullptr;
  sws_ctx = nullptr;

  stream_id = randomString(16);
}

ZDepthEncoder::~ZDepthEncoder() {
  if (frameCompressed != nullptr)
    delete frameCompressed;
  if (frameOriginal != nullptr)
    delete frameOriginal;
  if (paramsStruct != nullptr)
    delete paramsStruct;
  if (fd != nullptr)
    delete fd;
  if (sws_ctx != nullptr)
    sws_freeContext(sws_ctx);
}

void ZDepthEncoder::addFrameStruct(FrameStruct *fs) {
  frameOriginal = fs;

  if (frameOriginal == nullptr) {
    frameCompressed = nullptr;
  } else {

    if (frameCompressed == nullptr)
      frameCompressed = new FrameStruct();

    frameCompressed->deviceId = fs->deviceId;
    frameCompressed->frameDataType = 1;
    frameCompressed->frameId = totalCurrentFrameCounter;
    frameCompressed->frameType = fs->frameType;
    frameCompressed->messageType = fs->messageType;
    frameCompressed->sensorId = fs->sensorId;
    frameCompressed->streamId = stream_id;
    frameCompressed->sceneDesc = fs->sceneDesc;
    frameCompressed->timestamps.clear();
    frameCompressed->timestamps = std::vector<unsigned long>();
    frameCompressed->timestamps.push_back(frameOriginal->timestamps.front());
    frameCompressed->timestamps.push_back(currentTimeMs());

    uint16_t *data = nullptr;

    AVFrame *pFrameO = nullptr;
    AVFrame *pFrame = nullptr;
    cv::Mat img;

    if (fs->frameDataType == 0) {

      pFrameO = av_frame_alloc();
      pFrame = av_frame_alloc();

      id.imageBufferToAVFrame(fs, pFrameO);

      width = pFrameO->width;
      height = pFrameO->height;

      pFrame->format = AV_PIX_FMT_GBRP16LE;

      pFrame->width = width;
      pFrame->height = height;
      av_frame_get_buffer(pFrame, 0);

      if (sws_ctx == nullptr)
        sws_ctx = sws_getContext(width, height, (AVPixelFormat)pFrameO->format,
                                 width, height, (AVPixelFormat)pFrame->format,
                                 SWS_BILINEAR, NULL, NULL, NULL);

      sws_scale(sws_ctx, (const uint8_t *const *)pFrameO->data,
                pFrameO->linesize, 0, pFrameO->height, pFrame->data,
                pFrame->linesize);

      data = reinterpret_cast<uint16_t *>(&pFrame->data[0][0]);

    } else if (fs->frameDataType == 1) {

      if (fd == nullptr) {
        fd = new LibAvDecoder();
        fd->init(fs->codec_data.getParams());
      }

      img = fd->decode(fs);

      width = img.cols;
      height = img.rows;

      if (frameCompressed->frameType == 0) {
        cv::cvtColor(img, img, CV_BGR2BGRA);
      }

      data = reinterpret_cast<uint16_t *>(img.data);

    } else if (fs->frameDataType == 2 || fs->frameDataType == 3) {
      // fs->frame[8] ignores width and height set at [0] and [4] by
      // KinectReader
      memcpy(&width, &fs->frame[0], sizeof(int));
      memcpy(&height, &fs->frame[4], sizeof(int));
      data = reinterpret_cast<uint16_t *>(&fs->frame[8]);
    }

    if (paramsStruct == nullptr)
      getCodecParamsStruct();

    compressed.clear();
    compressor.Compress(width, height, data, compressed,
                        totalCurrentFrameCounter == 0);

    frameCompressed->codec_data = *paramsStruct;
    frameCompressed->frame = std::vector<unsigned char>(
        compressed.data(), compressed.data() + compressed.size());
    frameCompressed->timestamps.push_back(currentTimeMs());
    totalCurrentFrameCounter++;

    if (pFrame != nullptr) {
      av_frame_free(&pFrame);
      av_frame_free(&pFrameO);
    }
  }
}

void ZDepthEncoder::nextPacket() {
  if (frameOriginal != nullptr)
    delete frameOriginal;
  frameOriginal = nullptr;
  frameCompressed = nullptr;
}

bool ZDepthEncoder::hasNextPacket() { return true; }

FrameStruct *ZDepthEncoder::currentFrameEncoded() { return frameCompressed; }

FrameStruct *ZDepthEncoder::currentFrameOriginal() { return frameOriginal; }

CodecParamsStruct *ZDepthEncoder::getCodecParamsStruct() {
  if (paramsStruct == NULL) {
    paramsStruct = new CodecParamsStruct();
    paramsStruct->type = 2;
    paramsStruct->data.resize(4 + 4);

    memcpy(&paramsStruct->data[0], &width, sizeof(int));
    memcpy(&paramsStruct->data[4], &height, sizeof(int));
  }
  return paramsStruct;
}

uint ZDepthEncoder::getFps() { return fps; }
