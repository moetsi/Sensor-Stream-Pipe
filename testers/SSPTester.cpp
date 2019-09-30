//
// Created by amourao on 07/08/19.
//

#include <chrono>
#include <iostream>
#include <thread>
#include <unistd.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/log.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

#include "../encoders/LibAvEncoder.h"
#include "../structs/FrameStruct.hpp"
#include <cv.hpp>

#include "../decoders/IDecoder.h"
#include "../decoders/LibAvDecoder.h"
#include "../decoders/NvDecoder.h"
#include "../encoders/NullEncoder.h"
#include "../encoders/NvEncoder.h"
#include "../encoders/ZDepthEncoder.h"
#include "../readers/KinectReader.h"
#include "../readers/VideoFileReader.h"
#include "../utils/KinectUtils.h"
#include "../utils/SimilarityMeasures.h"
#include "../utils/Utils.h"
#include "../utils/VideoUtils.h"

int main(int argc, char *argv[]) {

  spdlog::set_level(spdlog::level::debug);
  av_log_set_level(AV_LOG_QUIET);

  srand(time(NULL) * getpid());
  // srand(getpid());

  std::unordered_map<std::string, IDecoder *> decoders;

  std::unordered_map<std::string, uint64_t> total_latencies;
  std::unordered_map<std::string, uint64_t> original_sizes;
  std::unordered_map<std::string, uint64_t> compressed_sizes;

  std::unordered_map<std::string, double> psnrs;
  std::unordered_map<std::string, double> mses;
  std::unordered_map<std::string, double> mse_comps;
  std::unordered_map<std::string, cv::Scalar> mssims;
  std::unordered_map<std::string, int> counts;
  std::unordered_map<std::string, int> rows;
  std::unordered_map<std::string, int> cols;
  std::unordered_map<std::string, uint> typesMap;
  std::unordered_map<std::string, uint> fps;

  av_register_all();

  if (argc < 3) {
    std::cerr << "Usage: ssp_tester <codec parameters> <time in seconds>"
              << std::endl;
    return 1;
  }
  std::string codec_parameters_file = std::string(argv[1]);
  uint time_in_seconds = std::stoul(argv[2]);
  uint stopAfter = time_in_seconds;
  uint i = 0;

  YAML::Node codec_parameters = YAML::LoadFile(codec_parameters_file);

  IReader *reader = nullptr;
  YAML::Node general_parameters = codec_parameters["general"];

  bool show_graphical_output = true;
  if (general_parameters["show_graphical_output"].IsDefined())
    show_graphical_output =
        general_parameters["show_graphical_output"].as<bool>();

  setupLogging(general_parameters);

  std::string reader_type =
      general_parameters["frame_source"]["type"].as<std::string>();
  if (reader_type == "frames") {
    reader =
        new ImageReader(general_parameters["frame_source"]["parameters"]["path"]
                            .as<std::string>());
  } else if (reader_type == "video") {
    std::string path = general_parameters["frame_source"]["parameters"]["path"]
                           .as<std::string>();
    if (general_parameters["frame_source"]["parameters"]["streams"]
            .IsDefined()) {
      std::vector<uint> streams =
          general_parameters["frame_source"]["parameters"]["streams"]
              .as<std::vector<uint>>();
      reader = new VideoFileReader(path, streams);
    } else {
      reader = new VideoFileReader(path);
    }
  } else if (reader_type == "kinect") {
    ExtendedAzureConfig c = buildKinectConfigFromYAML(
        general_parameters["frame_source"]["parameters"]);
    reader = new KinectReader(0, c);
  } else {
    spdlog::error("Unknown reader type: \"{}\". Supported types are "
                  "\"frames\", \"video\" and \"kinect\"",
                  reader_type);
    exit(1);
  }

  // TODO: use smarter pointers
  std::unordered_map<uint, IEncoder *> encoders;

  std::vector<uint> types = reader->getType();

  for (uint type : types) {
    YAML::Node v = codec_parameters["video_encoder"][type];
    std::string encoder_type = v["type"].as<std::string>();
    IEncoder *fe = nullptr;
    if (encoder_type == "libav")
      fe = new LibAvEncoder(v, reader->getFps());
    else if (encoder_type == "nvenc")
      fe = new NvEncoder(v, reader->getFps());
    else if (encoder_type == "zdepth")
      fe = new ZDepthEncoder(reader->getFps());
    else if (encoder_type == "null")
      fe = new NullEncoder(reader->getFps());
    else {
      spdlog::error("Unknown encoder type: \"{}\". Supported types are "
                    "\"libav\", \"nvenc\", \"zdepth\" and \"null\"",
                    encoder_type);
      exit(1);
    }
    encoders[type] = fe;
  }

  std::queue<FrameStruct> buffer;

  // This class only reads the file once
  // while ((currentTimeMs() - start_time) <= time_in_seconds * 1000) {
  while (stopAfter > 0) {

    std::vector<FrameStruct> v;

    // TODO: document what is happening with the Encoders and Queue
    while (v.empty()) {

      std::vector<FrameStruct *> frameStructs = reader->currentFrame();
      for (FrameStruct *frameStruct : frameStructs) {
        IEncoder *frameEncoder = encoders[frameStruct->frameType];

        frameEncoder->addFrameStruct(frameStruct);
        if (frameEncoder->hasNextPacket()) {

          FrameStruct f = FrameStruct(*frameEncoder->currentFrameEncoded());
          FrameStruct fo = FrameStruct(*frameEncoder->currentFrameOriginal());

          std::string decoder_id = f.streamId + std::to_string(f.sensorId);

          if (original_sizes.find(decoder_id) == original_sizes.end()) {
            original_sizes[decoder_id] = 0;
            compressed_sizes[decoder_id] = 0;
            total_latencies[decoder_id] = 0;

            psnrs[decoder_id] = 0;
            mses[decoder_id] = 0;
            mse_comps[decoder_id] = 0;

            counts[decoder_id] = 0;

            typesMap[decoder_id] = f.frameType;

            fps[decoder_id] = reader->getFps();
          }

          original_sizes[decoder_id] += fo.frame.size();
          compressed_sizes[decoder_id] += f.frame.size();
          total_latencies[decoder_id] +=
              f.timestamps.back() - f.timestamps.at(1);
          v.push_back(f);
          buffer.push(fo);

          frameEncoder->nextPacket();
        }
      }
      if (!reader->hasNextFrame()) {
        stopAfter--;
        reader->reset();
      }
      reader->nextFrame();
    }

    spdlog::debug("Message {} processed", i++);

    for (FrameStruct f : v) {
      std::string decoder_id = f.streamId + std::to_string(f.sensorId);
      cv::Mat img;
      bool imgChanged = frameStructToMat(f, img, decoders);
      if (!img.empty() && imgChanged) {

        cols[decoder_id] = img.cols;
        rows[decoder_id] = img.rows;

        FrameStruct fo = buffer.front();
        buffer.pop();
        cv::Mat frameOri;
        cv::Mat frameDiff;

        frameStructToMat(fo, frameOri, decoders);
        fo.frame.clear();

        frameOri = frameOri.clone();
        img = img.clone();

        if (frameOri.channels() == 4)
          cv::cvtColor(frameOri, frameOri, COLOR_BGRA2BGR);
        if (img.channels() == 4)
          cv::cvtColor(img, img, COLOR_BGRA2BGR);

        if (typesMap[decoder_id] == 0) {
          if (img.channels() == 3)
            psnrs[decoder_id] += getPSNR(frameOri, img, MAX_DEPTH_VALUE_8_BITS);
          else
            psnrs[decoder_id] +=
                getPSNR(frameOri, img, MAX_DEPTH_VALUE_12_BITS);
          mssims[decoder_id] += getMSSIM(frameOri, img);
          mses[decoder_id] += getMSE(frameOri, img);
        } else {
          mses[decoder_id] += getMSE(frameOri, img);
          cv::Mat imgA, frameOriA;
          img.convertTo(imgA, CV_16U);
          frameOri.convertTo(frameOriA, CV_16U);
          minMaxFilter<ushort>(imgA, imgA, 0, MAX_DEPTH_VALUE_12_BITS);
          minMaxFilter<ushort>(frameOriA, frameOriA, 0,
                               MAX_DEPTH_VALUE_12_BITS);
          mse_comps[decoder_id] += getMSE(frameOriA, imgA);
        }

        if (show_graphical_output) {
          if (f.frameType == 0) {

            absdiff(frameOri, img, frameDiff);

          } else if (f.frameType == 1) {

            absdiff(frameOri, img, frameDiff);

            if (img.type() == CV_16U) {
              // Compress images to show up on a 255 valued color map
              img *= (MAX_DEPTH_VALUE_8_BITS / (float)MAX_DEPTH_VALUE_12_BITS);
            }

            if (frameOri.type() == CV_16U) {
              // Compress images to show up on a 255 valued color map
              frameOri *=
                  (MAX_DEPTH_VALUE_8_BITS / (float)MAX_DEPTH_VALUE_12_BITS);
            }

            img.convertTo(img, CV_8U);
            frameOri.convertTo(frameOri, CV_8U);
            cv::applyColorMap(img, img, cv::COLORMAP_JET);
            cv::applyColorMap(frameOri, frameOri, cv::COLORMAP_JET);
          }
          if (f.frameType == 2) {
            absdiff(frameOri, img, frameDiff);

            double max = 1024;
            img *= (MAX_DEPTH_VALUE_8_BITS / (float)max);
            img.convertTo(img, CV_8U);

            frameOri *= (MAX_DEPTH_VALUE_8_BITS / (float)max);
            frameOri.convertTo(frameOri, CV_8U);
          }

          if (frameDiff.channels() == 1) {
            frameDiff.convertTo(frameDiff, CV_8UC1);
            cv::applyColorMap(frameDiff, frameDiff, cv::COLORMAP_HOT);
          }

          cv::namedWindow("Original " + decoder_id);
          cv::imshow("Original " + decoder_id, frameOri);
          cv::namedWindow("Encoded " + decoder_id);
          cv::imshow("Encoded " + decoder_id, img);
          cv::namedWindow("Diff " + decoder_id);
          cv::imshow("Diff " + decoder_id, frameDiff);

          cv::waitKey(1);
        }
        counts[decoder_id]++;
      }
    }
  }
  for (auto &it : typesMap) {
    // Do stuff
    std::string decoder_id = it.first;

    spdlog::critical("[statistics];[{}]", typesMap[decoder_id]);

    double time = counts[decoder_id] / (double)fps[decoder_id];
    spdlog::critical("\t[time];[{}];{};seconds", typesMap[decoder_id], time);
    spdlog::critical("\t[original_size];[{}];{};bytes", typesMap[decoder_id],
                     original_sizes[decoder_id]);
    spdlog::critical("\t[compressed_size];[{}];{};bytes", typesMap[decoder_id],
                     compressed_sizes[decoder_id]);

    spdlog::critical("\t[original_bandwidth];[{}];{};Mbps",
                     typesMap[decoder_id],
                     (original_sizes[decoder_id] * 8.0 / 1000000) / (time));
    spdlog::critical("\t[compressed_bandwidth];[{}];{};Mbps",
                     typesMap[decoder_id],
                     (compressed_sizes[decoder_id] * 8.0 / 1000000) / (time));

    spdlog::critical("\t[compression ratio];[{}];{};x", typesMap[decoder_id],
                     original_sizes[decoder_id] /
                         (float)compressed_sizes[decoder_id]);

    spdlog::critical("\t[latency];[{}];{};ms", typesMap[decoder_id],
                     total_latencies[decoder_id] / (float)counts[decoder_id]);

    if (typesMap[decoder_id] == 0) {
      spdlog::critical("\t[PSNR];[{}];{}", typesMap[decoder_id],
                       psnrs[decoder_id] / counts[decoder_id]);
      cv::Vec<double, 4> mssimV = (mssims[decoder_id] / counts[decoder_id]);
      spdlog::critical("\t[MSSIM];[{}];{};{};{};{}", typesMap[decoder_id],
                       mssimV(0), mssimV(1), mssimV(2), mssimV(3));
    } else {
      spdlog::critical(
          "\t[MSE];[{}];{}", typesMap[decoder_id],
          mses[decoder_id] /
              (counts[decoder_id] * cols[decoder_id] * rows[decoder_id]));
      spdlog::critical(
          "\t[MSE_4096];[{}];{}", typesMap[decoder_id],
          mse_comps[decoder_id] /
              (counts[decoder_id] * cols[decoder_id] * rows[decoder_id]));
    }
  }
  delete reader;
  for (auto const &x : encoders)
    delete x.second;

  for (auto const &x : decoders)
    delete x.second;

  return 0;
}