//
// Created by amourao on 27-06-2019.
//

#pragma once

#include <fstream>
#include <iostream>
#include <vector>

#include "../utils/logger.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

#include <cereal/archives/binary.hpp>

#include "../structs/frame_struct.hpp"
#include "../utils/utils.h"
#include "ireader.h"

class VideoFileReader : public IReader {
private:
  unsigned int fps_;
  std::string filename_;

  std::vector<unsigned int> video_stream_indexes_;
  bool video_stream_indexes_from_file_;
  std::vector<FrameStruct *> frame_structs_;
  FrameStruct *frame_struct_buffer_;
  FrameStruct frame_struct_template_;

  int current_frame_counter_;

  AVFormatContext *av_format_context_;

  std::unordered_map<unsigned int, CodecParamsStruct> codec_params_structs_;
  std::unordered_map<unsigned int, AVCodecContext *> av_codec_contexts_;

  AVPacket *packet_;

  bool libav_ready_;

  bool eof_reached_;

  void Init(std::string &filename);

public:
  VideoFileReader(std::string &filename);
  VideoFileReader(std::string &filename,
                  std::vector<unsigned int> &video_stream_indexes);

  ~VideoFileReader();

  void Reset();

  void GoToFrame(unsigned int frame_id);

  bool HasNextFrame();

  void NextFrame();

  std::vector<unsigned int> GetType();

  std::vector<FrameStruct *> GetCurrentFrame();

  unsigned int GetCurrentFrameId();

  unsigned int GetFps();
};
