//
// Created by amourao on 07/08/19.
//

#include <chrono>
#include <iostream>
#include <thread>
#include <unistd.h>

#include <zmq.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

#include "../encoders/FrameEncoder.h"
#include "../structs/FrameStruct.hpp"
#include <cv.hpp>

#include "../encoders/NullEncoder.h"
#include "../encoders/NvEncoder.h"
#include "../utils/SimilarityMeasures.h"
#include "../utils/Utils.h"
#include "../utils/VideoUtils.h"

int main(int argc, char *argv[]) {
  srand(time(NULL) * getpid());
  // srand(getpid());

  double psnr = 0, mmse = 0;
  cv::Scalar mssim;
  int i = 0;
  int j = 0;

  uint64_t original_size = 0;
  uint64_t compressed_size = 0;

  std::unordered_map<std::string, AVCodec *> pCodecs;
  std::unordered_map<std::string, AVCodecContext *> pCodecContexts;
  std::unordered_map<std::string, AVCodecParameters *> pCodecParameters;

  AVPacket *pPacket = av_packet_alloc();
  AVFrame *pFrame = av_frame_alloc();

  cv::Mat img;
  bool imgChanged = false;

  av_register_all();

  if (argc < 3) {
    std::cerr << "Usage: video_encoder_test <frame_file> <codec parameters>"
              << std::endl;
    return 1;
  }
  std::string frame_file = std::string(argv[1]);
  std::string codec_parameters_file = std::string(argv[2]);

  IReader *reader = new FrameReader(frame_file);

  YAML::Node codec_parameters = YAML::LoadFile(codec_parameters_file);
  YAML::Node v = codec_parameters["video_encoder"][reader->getType().at(0)];

  IEncoder *frameEncoder = new FrameEncoder(v, reader->getFps());
  std::string encoder_type = v["type"].as<std::string>();
  IEncoder *fe;
  if (encoder_type == "libav")
    fe = new FrameEncoder(v, reader->getFps());
  else if (encoder_type == "nvenc")
    fe = new NvEncoder(v, reader->getFps());
  else if (encoder_type == "null")
    fe = new NullEncoder(reader->getFps());

  // This class only reads the file once
  while (reader->hasNextFrame() || frameEncoder->hasNextPacket()) {

    while (!frameEncoder->hasNextPacket()) {
      FrameStruct *f = reader->currentFrame().front();
      original_size += f->frame.size();
      frameEncoder->addFrameStruct(f);
      reader->nextFrame();
    }

    std::vector<FrameStruct *> vO;
    vO.push_back(frameEncoder->currentFrameEncoded());
    FrameStruct f = *vO.at(0);

    compressed_size += f.frame.size();
    std::vector<FrameStruct> v;
    v.push_back(f);

    if (pCodecs.find(f.streamId + std::to_string(f.sensorId)) ==
        pCodecs.end()) {
      prepareDecodingStruct(&f, pCodecs, pCodecContexts, pCodecParameters);
    }

    AVCodecContext *pCodecContext =
        pCodecContexts[f.streamId + std::to_string(f.sensorId)];

    pPacket->data = &f.frame[0];
    pPacket->size = f.frame.size();

    int response = avcodec_send_packet(pCodecContext, pPacket);
    // std::cout << "avcodec_send_packet: " << response << " " <<
    // av_err2str(response) << std::endl;
    while (response >= 0) {
      // Return decoded output data (into a frame) from a decoder
      response = avcodec_receive_frame(pCodecContext, pFrame);
      // std::cout << "avcodec_receive_frame: " << response << " " <<
      // av_err2str(response) << std::endl;
      if (response == AVERROR(EAGAIN) || response == AVERROR_EOF) {
        break;
      }

      if (response >= 0) {
        i++;

        if (pCodecContext->pix_fmt == AV_PIX_FMT_GRAY12LE) {
          avframeToMatGray(pFrame, img);
        } else {
          avframeToMatYUV(pFrame, img);
        }

        imgChanged = true;
      }
    }
    std::cout << f.deviceId << ";" << f.sensorId << ";" << f.frameId
              << " received; size " << f.frame.size() << std::endl;

    if (imgChanged) {
      FrameStruct fo = *frameEncoder->currentFrameOriginal();
      cv::Mat frameOri = cv::imdecode(fo.frame, CV_LOAD_IMAGE_UNCHANGED);
      cv::Mat frameDiff;
      if (pCodecContext->pix_fmt == AV_PIX_FMT_GRAY12LE) {

        cv::Mat frameOriSquached;
        minMaxFilter<ushort>(frameOri, frameOriSquached, 0,
                             MAX_DEPTH_VALUE_12_BITS);
        psnr += getPSNR(frameOriSquached, img, MAX_DEPTH_VALUE_12_BITS);
        mmse += getMSE(frameOriSquached, img);
        img *= (MAX_DEPTH_VALUE_8_BITS / (float)MAX_DEPTH_VALUE_12_BITS);
        frameOri *= (MAX_DEPTH_VALUE_8_BITS / (float)MAX_DEPTH_VALUE_12_BITS);

        img.convertTo(img, CV_8U);
        frameOri.convertTo(frameOri, CV_8U);

        absdiff(frameOri, img, frameDiff);
        mssim += getMSSIM(frameOri, img);

        cv::applyColorMap(img, img, cv::COLORMAP_JET);
        cv::applyColorMap(frameOri, frameOri, cv::COLORMAP_JET);
        cv::applyColorMap(frameDiff, frameDiff, cv::COLORMAP_HOT);
      } else if (f.frameType == 1) {

        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);

        img.convertTo(img, CV_16U);

        cv::Mat frameOriSquached;
        minMaxFilter<ushort>(frameOri, frameOriSquached, 0,
                             MAX_DEPTH_VALUE_12_BITS);
        psnr += getPSNR(frameOriSquached, img, MAX_DEPTH_VALUE_12_BITS);
        mssim += getMSSIM(frameOriSquached, img);
        mmse += getMSE(frameOriSquached, img);
        img.convertTo(img, CV_8U);

        frameOri *= (MAX_DEPTH_VALUE_8_BITS / (float)MAX_DEPTH_VALUE_12_BITS);

        frameOri.convertTo(frameOri, CV_8U);

        absdiff(frameOri, img, frameDiff);

        cv::applyColorMap(img, img, cv::COLORMAP_JET);
        cv::applyColorMap(frameOri, frameOri, cv::COLORMAP_JET);
        cv::applyColorMap(frameDiff, frameDiff, cv::COLORMAP_HOT);

      } else {

        absdiff(frameOri, img, frameDiff);
        psnr += getPSNR(frameOri, img, MAX_DEPTH_VALUE_8_BITS);
        mssim += getMSSIM(frameOri, img);
        mmse += getMSE(frameOri, img);
      }

      cv::namedWindow("Original");
      cv::imshow("Original", frameOri);
      cv::namedWindow("Encoded");
      cv::imshow("Encoded", img);
      cv::namedWindow("Diff");

      cv::imshow("Diff", frameDiff);

      cv::waitKey(1);
      imgChanged = false;
    }

    for (uint i = 0; i < v.size(); i++) {
      FrameStruct f = v.at(i);
      FrameStruct *fO = vO.at(i);
      ;
      fO->frame.clear();
      delete fO;
    }

    frameEncoder->nextPacket();
  }

  delete frameEncoder;

  std::cout << "Avg PSNR: " << psnr / i << std::endl;
  std::cout << "Avg MSSIM: " << mssim / i << std::endl;
  std::cout << "Avg MSE: " << mmse / i << " "
            << " " << mmse / (i * img.cols * img.rows) << std::endl;
  std::cout << "original_size: " << original_size << " bytes" << std::endl;
  std::cout << "compressed_size: " << compressed_size << " bytes, bitrate: "
            << (8 * compressed_size) / (1000000.0 * reader->getFps()) << " Mbps"
            << std::endl;
  std::cout << "ratio: " << original_size / compressed_size << "x" << std::endl;

  return 0;
}