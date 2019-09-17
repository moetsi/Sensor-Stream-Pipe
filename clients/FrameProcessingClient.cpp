//
// Created by amourao on 26-06-2019.
//

#include <chrono>
#include <iostream>
#include <thread>
#include <unistd.h>

#include <opencv2/imgproc.hpp>
#include <zmq.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

#include "../decoders/FrameDecoder.h"
#include "../decoders/IDecoder.h"
#include "../decoders/NvDecoder.h"
#include "../readers/FrameReader.h"
#include "../structs/FrameStruct.hpp"
#include "../utils/Utils.h"
#include "../utils/VideoUtils.h"

int main(int argc, char *argv[]) {

  srand(time(NULL) * getpid());

  try {
    if (argc != 2) {
      std::cerr << "Usage: client <port>" << std::endl;
      return 1;
    }
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_PULL);
    socket.bind("tcp://*:" + std::string(argv[1]));

    uint64_t last_time = currentTimeMs();
    uint64_t start_time = last_time;
    uint64_t rec_frames = 0;
    double rec_mbytes = 0;

    std::unordered_map<std::string, double> rec_mbytes_per_stream;

    std::unordered_map<std::string, IDecoder *> decoders;

    bool imgChanged = false;

    for (;;) {
      zmq::message_t request;

      socket.recv(&request);

      if (rec_frames == 0) {
        last_time = currentTimeMs();
        start_time = last_time;
      }

      rec_frames += 1;
      uint64_t diff_time = currentTimeMs() - last_time;
      double diff_start_time =
          (currentTimeMs() - start_time) / (double)rec_frames;
      int64_t avg_fps;
      if (diff_start_time == 0)
        avg_fps = -1;
      else
        avg_fps = 1000 / diff_start_time;

      last_time = currentTimeMs();

      std::string result =
          std::string(static_cast<char *>(request.data()), request.size());

      std::vector<FrameStruct> f_list =
          parseCerealStructFromString<std::vector<FrameStruct>>(result);

      rec_mbytes += request.size() / 1000;

      for (int i = 0; i < f_list.size(); i++) {
        f_list.at(i).timestamps.push_back(currentTimeMs());
      }
      for (FrameStruct f : f_list) {
        std::string decoder_id = f.streamId + std::to_string(f.sensorId);

        cv::Mat img;
        rec_mbytes_per_stream[decoder_id] += f.frame.size() / 1000;
        if (f.frameDataType == 0) {
          img = cv::imdecode(f.frame, CV_LOAD_IMAGE_UNCHANGED);
          imgChanged = true;
        } else if (f.frameDataType == 2) {
          int rows, cols;
          memcpy(&cols, &f.frame[0], sizeof(int));
          memcpy(&rows, &f.frame[4], sizeof(int));
          img = cv::Mat(rows, cols, CV_8UC4, (void *)&f.frame[8],
                        cv::Mat::AUTO_STEP);
          imgChanged = true;
        } else if (f.frameDataType == 3) {
          int rows, cols;
          memcpy(&cols, &f.frame[0], sizeof(int));
          memcpy(&rows, &f.frame[4], sizeof(int));
          img = cv::Mat(rows, cols, CV_16UC1, (void *)&f.frame[8],
                        cv::Mat::AUTO_STEP);
          imgChanged = true;
        } else if (f.frameDataType == 1) {

          IDecoder *decoder;

          if (decoders.find(decoder_id) == decoders.end()) {
            CodecParamsStruct data = f.codec_data;
            if (data.type == 0) {
              FrameDecoder *fd = new FrameDecoder();
              fd->init(data.getParams());
              decoders[decoder_id] = fd;
            } else if (data.type == 1) {
              NvDecoder *fd = new NvDecoder();
              fd->init(data.data);
              decoders[decoder_id] = fd;
            }
          }

          decoder = decoders[decoder_id];

          img = decoder->decode(&f);
          imgChanged = true;

          f.frame.clear();
        }

        if (imgChanged && !img.empty()) {

          // Manipulate image to show as a color map
          if (f.frameType == 1) {
            if (img.type() == CV_16U) {
              // Compress images to show up on a 255 valued color map
              img *= (MAX_DEPTH_VALUE_8_BITS / (float)MAX_DEPTH_VALUE_12_BITS);
            }
            cv::Mat imgOut;

            img.convertTo(imgOut, CV_8U);
            cv::applyColorMap(imgOut, img, cv::COLORMAP_JET);
          }
          if (f.frameType == 2) {

            double max = 1024;
            img *= (MAX_DEPTH_VALUE_8_BITS / (float)max);
            img.convertTo(img, CV_8U);
          }

          cv::namedWindow(decoder_id);
          cv::imshow(decoder_id, img);
          cv::waitKey(1);
          imgChanged = false;
        }
      }

      std::cout << "Message received, took " << diff_time << " ms "
                << "; packet size " << request.size() << "; avg " << avg_fps
                << " fps; " << 8 * (rec_mbytes / (currentTimeMs() - start_time))
                << " avg Mbps"
                << " latency: " << (f_list.front().timestamps.at(0)) << " ms; "
                << (f_list.front().timestamps.back() -
                    f_list.front().timestamps.at(1))
                << " ms" << std::endl;

      for (FrameStruct f : f_list) {
        std::string decoder_id = f.streamId + std::to_string(f.sensorId);
        std::cout << "\t" << f.deviceId << ";" << f.sensorId << ";" << f.frameId
                  << " "
                  << 8 * (rec_mbytes_per_stream[decoder_id] /
                          (currentTimeMs() - start_time))
                  << " avg Mbps received"
                  << " latency: " << (f_list.front().timestamps.at(0))
                  << " ms; " << (f.timestamps.back() - f.timestamps.at(1))
                  << " ms" << std::endl;
      }
    }

  } catch (std::exception &e) {
    std::cerr << e.what() << std::endl;
  }

  // avformat_close_input(&pFormatContext);
  // avformat_free_context(pFormatContext);
  // av_packet_free(&pPacket);
  // av_frame_free(&pFrame);
  // avcodec_free_context(&pCodecContextEncoder);

  return 0;
}
