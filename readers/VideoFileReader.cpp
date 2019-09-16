//
// Created by amourao on 26-06-2019.
//

#include "VideoFileReader.h"

VideoFileReader::VideoFileReader(std::string &filename) {
  // get stream info from filename
  std::vector<std::string> valuesAll = split(filename, "-");
  std::vector<std::string> values = split(valuesAll[0], ".");

  currentFrameCounter = 0;
  eofReached = false;
  libAVReady = false;

  this->filename = filename;
}

VideoFileReader::~VideoFileReader() {
  avformat_close_input(&pFormatContext);
  avformat_free_context(pFormatContext);
  av_packet_free(&pPacket);
}

void VideoFileReader::init(std::string &filename) {
  av_register_all();
  std::cout << "initializing all the containers, codecs and protocols."
            << std::endl;

  pFormatContext = avformat_alloc_context();
  if (!pFormatContext) {
    std::cout << "ERROR could not allocate memory for Format Context"
              << std::endl;
    exit(1);
  }

  std::cout << "opening the input file and loading format (container) header"
            << filename << std::endl;
  // Open the file and read its header. The codecs are not opened.
  // The function arguments are:
  // AVFormatContext (the component we allocated memory for),
  // url (filename),
  // AVInputFormat (if you pass NULL it'll do the auto detect)
  // and AVDictionary (which are options to the demuxer)
  // http://ffmpeg.org/doxygen/trunk/group__lavf__decoding.html#ga31d601155e9035d5b0e7efedc894ee49
  if (avformat_open_input(&pFormatContext, filename.c_str(), NULL, NULL) != 0) {
    std::cerr << "ERROR could not open the file" << std::endl;
    exit(1);
  }

  std::cout << "format %s, duration %lld us, bit_rate %lld"
            << pFormatContext->iformat->name << " " << pFormatContext->duration
            << " " << pFormatContext->bit_rate << std::endl;

  std::cout << "finding stream info from format" << std::endl;
  // read Packets from the Format to get stream information
  // this function populates pFormatContext->streams
  // (of size equals to pFormatContext->nb_streams)
  // the arguments are:
  // the AVFormatContext
  // and options contains options for codec corresponding to i-th stream.
  // On return each dictionary will be filled with options that were not found.
  // https://ffmpeg.org/doxygen/trunk/group__lavf__decoding.html#gad42172e27cddafb81096939783b157bb
  if (avformat_find_stream_info(pFormatContext, NULL) < 0) {
    std::cout << "ERROR could not get the stream info" << std::endl;
    exit(-1);
  }

  // the component that knows how to enCOde and DECode the stream
  // it's the codec (audio or video)
  // http://ffmpeg.org/doxygen/trunk/structAVCodec.html

  // this component describes the properties of a codec used by the stream i
  // https://ffmpeg.org/doxygen/trunk/structAVCodecParameters.html

  // loop though all the streams and print its main information
  for (int i = 0; i < pFormatContext->nb_streams; i++) {
    AVCodecParameters *pCodecParameter = pFormatContext->streams[i]->codecpar;
    std::cout << "AVStream->time_base before open coded %d/%d"
              << " " << pFormatContext->streams[i]->time_base.num << " "
              << pFormatContext->streams[i]->time_base.den << std::endl;
    std::cout << "AVStream->r_frame_rate before open coded %d/%d"
              << " " << pFormatContext->streams[i]->r_frame_rate.num << " "
              << pFormatContext->streams[i]->r_frame_rate.den << std::endl;
    std::cout << "AVStream->start_time %" PRId64 << " "
              << pFormatContext->streams[i]->start_time << std::endl;
    std::cout << "AVStream->duration %" PRId64 << " "
              << pFormatContext->streams[i]->duration << std::endl;
    std::cout << "finding the proper decoder (CODEC)" << std::endl;

    // if (pCodecParameters->codec_id == AV_CODEC_ID_RAWVIDEO)
    //  pCodecParameters->format = AV_PIX_FMT_GBRP16LE;

    AVCodec *pCodec = avcodec_find_decoder(pCodecParameter->codec_id);
    if (pCodec == NULL) {
      std::cout << "ERROR unsupported codec!" << std::endl;
    } else if (pCodecParameter->codec_type == AVMEDIA_TYPE_VIDEO) {
      video_stream_indexes.emplace(i);
      std::cout << "Video Codec: resolution %d x %d"
                << " " << pCodecParameter->width << " "
                << pCodecParameter->height << std::endl;

      AVCodecContext *pCodecContext = avcodec_alloc_context3(pCodec);
      if (!pCodecContext) {
        std::cout << "failed to allocated memory for AVCodecContext"
                  << std::endl;
        exit(-1);
      }

      if (avcodec_parameters_to_context(pCodecContext, pCodecParameter) < 0) {
        std::cout << "failed to copy codec params to codec context"
                  << std::endl;
        exit(-1);
      }

      if (avcodec_open2(pCodecContext, pCodec, NULL) < 0) {
        std::cout << "failed to open codec through avcodec_open2" << std::endl;
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

      avcodec_parameters_free(&pCodecParameter);
    }
  }

  pPacket = av_packet_alloc();
  if (!pPacket) {
    std::cout << "failed to allocated memory for AVPacket" << std::endl;
    exit(-1);
  }

  // frameStruct.codec_data = videoReader.getCodecParamsStruct();
  frameStructTemplate.messageType = 0;

  frameStructTemplate.frameDataType = 1;
  frameStructTemplate.streamId = randomString(16);
  frameStructTemplate.deviceId = 0;

  libAVReady = true;
}

int VideoFileReader::currentFrameId() { return currentFrameCounter; }

/*
std::vector<unsigned char> VideoFileReader::currentFrameBytes() {
  return std::vector<unsigned char>(pPacket->data,
                                    pPacket->data + pPacket->size);
}
 */

void VideoFileReader::nextFrame() {
  if (!libAVReady)
    init(this->filename);

  if (frameStructsBuffer != nullptr) {
    frameStructs.clear();
    frameStructs.push_back(frameStructsBuffer);
    frameStructsBuffer = nullptr;
  }

  av_packet_unref(pPacket);
  int error = 0;
  while (error >= 0) {
    error = av_read_frame(pFormatContext, pPacket);
    // if it's the video stream
    if (video_stream_indexes.find(pPacket->stream_index) !=
        video_stream_indexes.end()) {

      FrameStruct *frameStruct = new FrameStruct(frameStructTemplate);
      frameStruct->frame = std::vector<unsigned char>(
          &pPacket->data[0], &pPacket->data[0] + pPacket->size);
      frameStruct->frameId = currentFrameCounter;
      frameStruct->sensorId = pPacket->stream_index;
      frameStruct->frameType = pPacket->stream_index;
      frameStruct->timestamps.push_back(pPacket->pts);
      frameStruct->codec_data = pCodecParameters[pPacket->stream_index];

      if (frameStructs.empty() ||
          (std::abs((long)(pPacket->pts -
                           (long)frameStructs.front()->timestamps.front())) <
           10000))
        frameStructs.push_back(frameStruct);
      else if (frameStructsBuffer != nullptr)
        std::cout << "ERROR" << std::endl;
      else {
        frameStructsBuffer = frameStruct;
        currentFrameCounter += 1;
        frameStructsBuffer->frameId = currentFrameCounter;
        break;
      }
    }

    if (error == AVERROR_EOF) {
      for (auto fs : frameStructs)
        delete fs;
      frameStructs.clear();
      if (frameStructsBuffer != nullptr) {
        delete frameStructsBuffer;
        frameStructsBuffer = nullptr;
      }
      error = 1;
      reset();
    }
    // https://ffmpeg.org/doxygen/trunk/group__lavc__packet.html#ga63d5a489b419bd5d45cfd09091cbcbc2
  }

  // std::cout << type << " Next frame: " << error << " " << av_err2str(error)
  // << " " << currentFrameCounter << std::endl;
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
    std::cerr << "Error " << error << " " << av_err2str(error) << std::endl;
  }
}

void VideoFileReader::reset() {
  currentFrameCounter = 0;
  eofReached = false;

  int error = av_seek_frame(pFormatContext, -1, 0, AVSEEK_FLAG_BACKWARD);

  if (error < 0) {
    std::cerr << "Error " << error << " " << av_err2str(error) << std::endl;
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
/*

 */