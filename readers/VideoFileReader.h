//
// Created by amourao on 27-06-2019.
//

#pragma once

#include <fstream>
#include <iostream>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

#include <cereal/archives/binary.hpp>

#include "../structs/FrameStruct.hpp"
#include "../utils/Utils.h"

class VideoFileReader {
private:
  unsigned int fps;
  std::string sceneDesc;
  std::string type;
  std::string filename;

  std::set<uint> video_stream_indexes;
  std::vector<FrameStruct *> frameStructs;
  FrameStruct *frameStructsBuffer;
  FrameStruct frameStructTemplate;

  int currentFrameCounter;

  AVFormatContext *pFormatContext;

  std::unordered_map<uint, CodecParamsStruct> pCodecParameters;
  std::unordered_map<uint, AVCodecContext *> pCodecContexts;

  AVPacket *pPacket;

  bool libAVReady;

  bool eofReached;

  void init(std::string &filename);

public:
  VideoFileReader(std::string &filename);

  ~VideoFileReader();

  void reset();

  void goToFrame(unsigned int frameId);

  bool hasNextFrame();

  void nextFrame();

  std::vector<FrameStruct *> currentFrame();

  int currentFrameId();

  unsigned int getFps();
};
