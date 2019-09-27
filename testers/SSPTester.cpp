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

  srand(time(NULL) * getpid());
  // srand(getpid());

  double psnr = 0;
  double mse = 0;
  double mse_comp = 0;
  cv::Scalar mssim;
  int i = 0;

  uint frameType = 0;

  uint64_t original_size = 0;
  uint64_t compressed_size = 0;

  std::unordered_map<std::string, IDecoder *> decoders;

  cv::Mat img;
  bool imgChanged = false;

  av_register_all();

  if (argc < 3) {
    std::cerr << "Usage: ssp_tester <codec parameters> <time in seconds>"
              << std::endl;
    return 1;
  }
  std::string codec_parameters_file = std::string(argv[1]);
  uint time_in_seconds = std::stoul(argv[2]);
  uint stopAfter = time_in_seconds;

  YAML::Node codec_parameters = YAML::LoadFile(codec_parameters_file);

  IReader *reader = nullptr;
  YAML::Node general_parameters = codec_parameters["general"];

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
    frameType = type;
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

          original_size += fo.frame.size();
          compressed_size += f.frame.size();
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

    FrameStruct f = v.front();

    imgChanged = frameStructToMat(f, img, decoders);

    f.frame.clear();
    spdlog::debug("\t{};{};{} received", f.deviceId, f.sensorId, f.frameId);

    if (!img.empty() && imgChanged) {
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

      if (f.frameType == 0) {
        if (img.channels() == 3)
          psnr += getPSNR(frameOri, img, MAX_DEPTH_VALUE_8_BITS);
        else
          psnr += getPSNR(frameOri, img, MAX_DEPTH_VALUE_12_BITS);
        mssim += getMSSIM(frameOri, img);
        mse += getMSE(frameOri, img);
      } else {
        mse += getMSE(frameOri, img);
        cv::Mat imgA, frameOriA;
        img.convertTo(imgA, CV_16U);
        frameOri.convertTo(frameOriA, CV_16U);
        minMaxFilter<ushort>(imgA, imgA, 0, MAX_DEPTH_VALUE_12_BITS);
        minMaxFilter<ushort>(frameOriA, frameOriA, 0, MAX_DEPTH_VALUE_12_BITS);
        mse_comp += getMSE(frameOriA, imgA);
      }

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
          frameOri *= (MAX_DEPTH_VALUE_8_BITS / (float)MAX_DEPTH_VALUE_12_BITS);
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

      cv::namedWindow("Original");
      cv::imshow("Original", frameOri);
      cv::namedWindow("Encoded");
      cv::imshow("Encoded", img);
      cv::namedWindow("Diff");

      cv::imshow("Diff", frameDiff);

      cv::waitKey(1);
      imgChanged = false;
      i++;
    }
  }

  spdlog::critical("original_size: {} bytes", original_size);
  spdlog::critical("compressed_size: {} bytes", compressed_size);
  spdlog::critical("compression ratio: {}x",
                   original_size / (float)compressed_size);

  if (frameType == 0) {
    spdlog::critical("Avg PSNR: {}", psnr / i);
    cv::Vec<double, 4> mssimV = (mssim / i);
    spdlog::critical("Avg MSSIM: ({};{};{};{})", mssimV(0), mssimV(1),
                     mssimV(2), mssimV(3));
  } else {
    spdlog::critical("Avg MSE per voxel: {}", mse / (i * img.cols * img.rows));
    spdlog::critical("Avg MSE per voxel (4096 mm): {}",
                     mse_comp / (i * img.cols * img.rows));
  }

  delete reader;
  for (auto const &x : encoders)
    delete x.second;

  for (auto const &x : decoders)
    delete x.second;

  return 0;
}