//
// Created by amourao on 26-06-2019.
//

#include "video_file_reader.h"

VideoFileReader::VideoFileReader(std::string &_filename) {
  currentFrameCounter = 0;
  eofReached = false;
  libAVReady = false;

  filename = _filename;

  video_stream_indexes_from_file = false;
}

VideoFileReader::VideoFileReader(std::string &_filename,
                                 std::vector<uint> &_video_stream_indexes) {
  currentFrameCounter = 0;
  eofReached = false;
  libAVReady = false;

  filename = _filename;

  video_stream_indexes = _video_stream_indexes;
  video_stream_indexes_from_file = true;
}

VideoFileReader::~VideoFileReader() {
  av_packet_unref(pPacket);
  av_packet_free(&pPacket);

  for (auto const &x : pCodecContexts){
    AVCodecContext * c = x.second;
    avcodec_free_context(&c);
  }

  avformat_close_input(&pFormatContext);


  for (auto fs : frameStructs)
    delete fs;
  if (frameStructsBuffer != nullptr)
    delete frameStructsBuffer;


}

void VideoFileReader::init(std::string &filename) {
  av_register_all();
  spdlog::info("VideoFileReader: initializing all the containers, codecs and "
               "protocols.");

  pFormatContext = avformat_alloc_context();
  if (!pFormatContext) {
    spdlog::error("Could not allocate memory for Format Context.");
    exit(1);
  }

  spdlog::info(
      "Opening the input file and loading format (container) header {}",
      filename);

  if (avformat_open_input(&pFormatContext, filename.c_str(), NULL, NULL) != 0) {
    spdlog::error("Could not open the file.");
    exit(1);
  }

  spdlog::info("format {}, duration {} us, bit_rate {}",
               pFormatContext->iformat->name, pFormatContext->duration,
               pFormatContext->bit_rate);

  spdlog::info("Finding stream info from format.");

  if (avformat_find_stream_info(pFormatContext, NULL) < 0) {
    spdlog::error("Could not get the stream info.");
    exit(-1);
  }

  // the component that knows how to enCOde and DECode the stream
  // it's the codec (audio or video)
  // http://ffmpeg.org/doxygen/trunk/structAVCodec.html

  // this component describes the properties of a codec used by the stream i
  // https://ffmpeg.org/doxygen/trunk/structAVCodecParameters.html

  // loop though all the streams and print its main information
  for (uint i = 0; i < pFormatContext->nb_streams; i++) {
    AVCodecParameters *pCodecParameter = pFormatContext->streams[i]->codecpar;
    spdlog::info("AVStream->time_base before open coded: {}/{}",
                 pFormatContext->streams[i]->time_base.num,
                 pFormatContext->streams[i]->time_base.den);
    spdlog::info("AVStream->r_frame_rate before open coded: {}/{}",
                 pFormatContext->streams[i]->r_frame_rate.num,
                 pFormatContext->streams[i]->r_frame_rate.den);

    spdlog::info("AVStream->start_time: {}",
                 pFormatContext->streams[i]->start_time);
    spdlog::info("AVStream->duration: {}",
                 pFormatContext->streams[i]->duration);

    spdlog::info("finding the proper decoder (CODEC)",
                 pFormatContext->streams[i]->duration);

    AVCodec *pCodec = avcodec_find_decoder(pCodecParameter->codec_id);
    if (pCodec == NULL) {
      spdlog::warn("Non video stream detected ({}), skipping", i);
    } else if (pCodecParameter->codec_type == AVMEDIA_TYPE_VIDEO) {
      spdlog::warn("Video stream detected ({})", i);
      if (!video_stream_indexes_from_file)
        video_stream_indexes.push_back(i);

      std::vector<uint>::iterator it;

      it = find(video_stream_indexes.begin(), video_stream_indexes.end(), i);
      if (it != video_stream_indexes.end()) {

        spdlog::info("Video Codec: resolution {}x{}", pCodecParameter->width,
                     pCodecParameter->height);

        AVCodecContext *pCodecContext = avcodec_alloc_context3(pCodec);
        if (!pCodecContext) {
          spdlog::error("Failed to allocated memory for AVCodecContext.");
          exit(-1);
        }

        if (avcodec_parameters_to_context(pCodecContext, pCodecParameter) < 0) {
          spdlog::error("Failed to copy codec params to codec context.");
          exit(-1);
        }

        if (avcodec_open2(pCodecContext, pCodec, NULL) < 0) {
          spdlog::error("Failed to open codec through avcodec_open2.");
          exit(-1);
        }

        pCodecContexts[i] = pCodecContext;
        frameStructsBuffer = nullptr;

        fps = av_q2d(
            pFormatContext->streams[video_stream_indexes.begin().operator*()]
                ->r_frame_rate);

        void *sEPointer = pCodecParameter->extradata;
        size_t sESize = pCodecParameter->extradata_size;
        size_t sSize = sizeof(*pCodecParameter);

        std::vector<unsigned char> e(sSize);
        std::vector<unsigned char> ed(sESize);

        memcpy(&e[0], pCodecParameter, sSize);
        memcpy(&ed[0], sEPointer, sESize);

        CodecParamsStruct codecParamsStruct(0, e, ed);
        pCodecParameters[i] = codecParamsStruct;

        //avcodec_parameters_free(&pCodecParameter);
      }
    }
  }

  pPacket = av_packet_alloc();
  if (!pPacket) {
    spdlog::error("Failed to allocated memory for AVPacket.");
    exit(-1);
  }

  frameStructTemplate.messageType = 0;

  frameStructTemplate.frameDataType = 1;
  frameStructTemplate.streamId = randomString(16);
  frameStructTemplate.deviceId = 0;

  libAVReady = true;
}

uint VideoFileReader::currentFrameId() { return currentFrameCounter; }

void VideoFileReader::nextFrame() {
  if (!libAVReady)
    init(this->filename);

  if (frameStructsBuffer != nullptr) {
    frameStructs.clear();
    frameStructs.push_back(frameStructsBuffer);
    frameStructsBuffer = nullptr;
  }

  int error = 0;
  while (error >= 0) {
    error = av_read_frame(pFormatContext, pPacket);
    // if it's the video stream
    std::vector<uint>::iterator it;

    it = find(video_stream_indexes.begin(), video_stream_indexes.end(),
              pPacket->stream_index);
    if (it != video_stream_indexes.end()) {

      FrameStruct *frameStruct = new FrameStruct(frameStructTemplate);
      frameStruct->frame = std::vector<unsigned char>(
          &pPacket->data[0], &pPacket->data[0] + pPacket->size);
      frameStruct->frameId = currentFrameCounter;
      frameStruct->sensorId = pPacket->stream_index;
      frameStruct->frameType = pPacket->stream_index;
      frameStruct->timestamps.push_back(pPacket->pts);
      frameStruct->timestamps.push_back(currentTimeMs());
      frameStruct->codec_data = pCodecParameters[pPacket->stream_index];

      if (frameStructs.empty() ||
          (std::abs((long)(pPacket->pts -
                           (long)frameStructs.front()->timestamps.front())) <
           10000)) {
        frameStructs.push_back(frameStruct);
      } else if (frameStructsBuffer == nullptr) {
        frameStructsBuffer = frameStruct;
        currentFrameCounter += 1;
        frameStructsBuffer->frameId = currentFrameCounter;
        av_packet_unref(pPacket);
        break;
      }
    }
    av_packet_unref(pPacket);
    if (error == AVERROR_EOF) {
      eofReached = true;
      for (auto fs : frameStructs)
        delete fs;
      frameStructs.clear();
      if (frameStructsBuffer != nullptr) {
        delete frameStructsBuffer;
        frameStructsBuffer = nullptr;
      }
      error = 1;
      break;
    }
  }

}

bool VideoFileReader::hasNextFrame() {
  if (!libAVReady)
    init(this->filename);
  return !eofReached;
}

void VideoFileReader::goToFrame(unsigned int frameId) {
  currentFrameCounter = frameId;
  eofReached = false;
  int error = av_seek_frame(pFormatContext, -1, frameId, AVSEEK_FLAG_FRAME);
  if (error < 0) {
    spdlog::error("Error seeking to frame {}: {}", frameId, av_err2str(error));
  }
}

void VideoFileReader::reset() {
  currentFrameCounter = 0;
  eofReached = false;

  int error = av_seek_frame(pFormatContext, -1, 0, AVSEEK_FLAG_BACKWARD);

  if (error < 0) {
    spdlog::error("Error seeking to frame {}: {}", 0, av_err2str(error));
  }
}

unsigned int VideoFileReader::getFps() {
  if (!libAVReady)
    init(this->filename);
  return fps;
}

std::vector<FrameStruct *> VideoFileReader::currentFrame() {
  return frameStructs;
}

std::vector<uint> VideoFileReader::getType() {
  if (!libAVReady)
    init(this->filename);
  return video_stream_indexes;
}
