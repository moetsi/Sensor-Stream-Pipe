//
// Created by amourao on 07/08/19.
//

#include <chrono>
#include <iostream>
#include <thread>
#ifdef _WIN32
#include <io.h>
#define ushort u_short
#else
#include <unistd.h>
#endif 

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/log.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

#include "../encoders/libav_encoder.h"
#include "../structs/frame_struct.hpp"
#include <cv.hpp>

#include "../decoders/idecoder.h"
#include "../decoders/libav_decoder.h"

#include "../encoders/null_encoder.h"
#include "../encoders/zdepth_encoder.h"
#include "../readers/video_file_reader.h"
#include "../readers/multi_image_reader.h"
#include "../utils/image_converter.h"
#include "../utils/similarity_measures.h"
#include "../utils/utils.h"
#include "../utils/video_utils.h"

#ifdef SSP_WITH_NVPIPE_SUPPORT
#include "../decoders/nv_decoder.h"
#include "../encoders/nv_encoder.h"
#endif

#ifdef SSP_WITH_KINECT_SUPPORT
#include "../readers/kinect_reader.h"
#include "../utils/kinect_utils.h"
#endif



int main(int argc, char *argv[]) {

  spdlog::set_level(spdlog::level::debug);
  av_log_set_level(AV_LOG_QUIET);

  srand(time(NULL) * getpid());
  // srand(getpid());

  std::unordered_map<std::string, std::shared_ptr<IDecoder>> decoders;

  std::unordered_map<std::string, int64_t> total_latencies;
  std::unordered_map<std::string, int64_t> original_sizes;
  std::unordered_map<std::string, int64_t> compressed_sizes;

  std::unordered_map<std::string, double> psnrs;
  std::unordered_map<std::string, double> mses;
  std::unordered_map<std::string, double> mse_comps;
  std::unordered_map<std::string, cv::Scalar> mssims;
  std::unordered_map<std::string, int> counts;
  std::unordered_map<std::string, int> rows;
  std::unordered_map<std::string, int> cols;
  std::unordered_map<std::string, unsigned int> types_map;
  std::unordered_map<std::string, unsigned int> fps;

  av_register_all();

  if (argc < 3) {
    std::cerr << "Usage: ssp_tester <codec parameters> <time in seconds>"
              << std::endl;
    return 1;
  }
  std::string codec_parameters_file = std::string(argv[1]);
  unsigned int time_in_seconds = std::stoul(argv[2]);
  unsigned int stop_after = time_in_seconds;
  unsigned int i = 0;

  YAML::Node codec_parameters = YAML::LoadFile(codec_parameters_file);

  YAML::Node general_parameters = codec_parameters["general"];

  bool show_graphical_output = true;
  if (general_parameters["show_graphical_output"].IsDefined())
    show_graphical_output =
        general_parameters["show_graphical_output"].as<bool>();

  SetupLogging(general_parameters);

  std::unique_ptr<IReader> reader;

  std::string reader_type =
      general_parameters["frame_source"]["type"].as<std::string>();
  if (reader_type == "frames") {
    if (general_parameters["frame_source"]["parameters"]["path"].IsSequence())
      reader = std::unique_ptr<MultiImageReader>(new MultiImageReader(
          general_parameters["frame_source"]["parameters"]["path"]
              .as<std::vector<std::string>>()));
    else
      reader = std::unique_ptr<ImageReader>(new ImageReader(
          general_parameters["frame_source"]["parameters"]["path"]
              .as<std::string>()));
  } else if (reader_type == "video") {
    std::string path = general_parameters["frame_source"]["parameters"]["path"]
                           .as<std::string>();
    if (general_parameters["frame_source"]["parameters"]["streams"]
            .IsDefined()) {
      std::vector<unsigned int> streams =
          general_parameters["frame_source"]["parameters"]["streams"]
              .as<std::vector<unsigned int>>();
      reader =
          std::unique_ptr<VideoFileReader>(new VideoFileReader(path, streams));
    } else {
      reader = std::unique_ptr<VideoFileReader>(new VideoFileReader(path));
    }

  } else if (reader_type == "kinect") {
#ifdef SSP_WITH_KINECT_SUPPORT
    ExtendedAzureConfig c = BuildKinectConfigFromYAML(
        general_parameters["frame_source"]["parameters"]);
    reader = std::unique_ptr<KinectReader>(new KinectReader(0, c));
#else
    exit(1);
#endif
  } else {
    spdlog::error("Unknown reader type: \"{}\". Supported types are "
                  "\"frames\", \"video\" and \"kinect\"",
                  reader_type);
    exit(1);
  }

  std::unordered_map<unsigned int, std::shared_ptr<IEncoder>> encoders;

  std::vector<unsigned int> types = reader->GetType();

  for (unsigned int type : types) {
    YAML::Node v = codec_parameters["video_encoder"][type];
    std::string encoder_type = v["type"].as<std::string>();
    std::shared_ptr<IEncoder> fe = nullptr;
    if (encoder_type == "libav")
      fe = std::shared_ptr<LibAvEncoder>(new LibAvEncoder(v, reader->GetFps()));
    else if (encoder_type == "nvenc") {
#ifdef SSP_WITH_NVPIPE_SUPPORT
      fe = std::shared_ptr<NvEncoder>(new NvEncoder(v, reader->GetFps()));
#else
      spdlog::error("SSP compiled without \"nvenc\" reader support. Set to "
                    "SSP_WITH_NVPIPE_SUPPORT=ON when configuring with cmake");
      exit(1);
#endif
    } else if (encoder_type == "zdepth")
      fe = std::shared_ptr<ZDepthEncoder>(new ZDepthEncoder(reader->GetFps()));
    else if (encoder_type == "null")
      fe = std::shared_ptr<NullEncoder>(new NullEncoder(reader->GetFps()));
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
  while (stop_after > 0) {

    std::vector<FrameStruct> v;

    // TODO: document what is happening with the Encoders and Queue
    while (v.empty()) {

      std::vector<std::shared_ptr<FrameStruct>> frame_structs =
          reader->GetCurrentFrame();
      for (std::shared_ptr<FrameStruct> frame_struct : frame_structs) {
        std::shared_ptr<IEncoder> frameEncoder =
            encoders[frame_struct->frame_type];

        frameEncoder->AddFrameStruct(frame_struct);
        if (frameEncoder->HasNextPacket()) {

          FrameStruct f = FrameStruct(*frameEncoder->CurrentFrameEncoded());
          FrameStruct fo = FrameStruct(*frameEncoder->CurrentFrameOriginal());

          std::string decoder_id = f.stream_id + std::to_string(f.sensor_id);

          if (original_sizes.find(decoder_id) == original_sizes.end()) {
            original_sizes[decoder_id] = 0;
            compressed_sizes[decoder_id] = 0;
            total_latencies[decoder_id] = 0;

            psnrs[decoder_id] = 0;
            mses[decoder_id] = 0;
            mse_comps[decoder_id] = 0;

            counts[decoder_id] = 0;

            types_map[decoder_id] = f.frame_type;

            fps[decoder_id] = reader->GetFps();
          }

          original_sizes[decoder_id] += fo.frame.size();
          compressed_sizes[decoder_id] += f.frame.size();
          total_latencies[decoder_id] +=
              f.timestamps.back() - f.timestamps.at(1);
          v.push_back(f);
          buffer.push(fo);

          frameEncoder->NextPacket();
        }
      }
      if (!reader->HasNextFrame()) {
        stop_after--;
        reader->Reset();
      }
      reader->NextFrame();
    }

    spdlog::debug("Message {} processed", i++);

    for (FrameStruct f : v) {
      std::string decoder_id = f.stream_id + std::to_string(f.sensor_id);
      cv::Mat img;
      bool img_changed = FrameStructToMat(f, img, decoders);
      if (!img.empty() && img_changed) {

        cols[decoder_id] = img.cols;
        rows[decoder_id] = img.rows;

        FrameStruct fo = buffer.front();

        cv::Mat frame_ori;
        cv::Mat frame_diff;

        FrameStructToMat(fo, frame_ori, decoders);
        img = img.clone();

        buffer.pop();

        if (frame_ori.channels() == 4)
          cv::cvtColor(frame_ori, frame_ori, COLOR_BGRA2BGR);
        if (img.channels() == 4)
          cv::cvtColor(img, img, COLOR_BGRA2BGR);

        if (types_map[decoder_id] == 0) {
          if (img.channels() == 3)
            psnrs[decoder_id] +=
                GetPSNR(frame_ori, img, MAX_DEPTH_VALUE_8_BITS);
          else
            psnrs[decoder_id] +=
                GetPSNR(frame_ori, img, MAX_DEPTH_VALUE_12_BITS);
          mssims[decoder_id] += GetMSSIM(frame_ori, img);
          mses[decoder_id] += GetMSE(frame_ori, img);
        } else {
          mses[decoder_id] += GetMSE(frame_ori, img);
          cv::Mat img_tmp, frame_ori_tmp;
          img.convertTo(img_tmp, CV_16U);
          frame_ori.convertTo(frame_ori_tmp, CV_16U);
          MinMaxFilter<ushort>(img_tmp, img_tmp, 0, MAX_DEPTH_VALUE_12_BITS);
          MinMaxFilter<ushort>(frame_ori_tmp, frame_ori_tmp, 0,
                               MAX_DEPTH_VALUE_12_BITS);
          mse_comps[decoder_id] += GetMSE(frame_ori_tmp, img_tmp);
        }

        if (show_graphical_output) {
          if (f.frame_type == 0) {

            absdiff(frame_ori, img, frame_diff);

          } else if (f.frame_type == 1) {

            absdiff(frame_ori, img, frame_diff);

            if (img.type() == CV_16U) {
              // Compress images to show up on a 255 valued color map
              img *= (MAX_DEPTH_VALUE_8_BITS / (float)MAX_DEPTH_VALUE_12_BITS);
            }

            if (frame_ori.type() == CV_16U) {
              // Compress images to show up on a 255 valued color map
              frame_ori *=
                  (MAX_DEPTH_VALUE_8_BITS / (float)MAX_DEPTH_VALUE_12_BITS);
            }

            img.convertTo(img, CV_8U);
            frame_ori.convertTo(frame_ori, CV_8U);
            cv::applyColorMap(img, img, cv::COLORMAP_JET);
            cv::applyColorMap(frame_ori, frame_ori, cv::COLORMAP_JET);
          }
          if (f.frame_type == 2) {
            absdiff(frame_ori, img, frame_diff);

            double max = 1024;
            img *= (MAX_DEPTH_VALUE_8_BITS / (float)max);
            img.convertTo(img, CV_8U);

            frame_ori *= (MAX_DEPTH_VALUE_8_BITS / (float)max);
            frame_ori.convertTo(frame_ori, CV_8U);
          }

          if (frame_diff.channels() == 1) {
            frame_diff.convertTo(frame_diff, CV_8UC1);
            cv::applyColorMap(frame_diff, frame_diff, cv::COLORMAP_HOT);
          }

          cv::namedWindow("Original " + decoder_id);
          cv::imshow("Original " + decoder_id, frame_ori);
          cv::namedWindow("Encoded " + decoder_id);
          cv::imshow("Encoded " + decoder_id, img);
          cv::namedWindow("Diff " + decoder_id);
          cv::imshow("Diff " + decoder_id, frame_diff);

          cv::waitKey(1);
        }
        counts[decoder_id]++;
      }
    }
  }
  for (auto &it : types_map) {
    // Do stuff
    std::string decoder_id = it.first;

    spdlog::critical("[statistics];[{}]", types_map[decoder_id]);

    double time = counts[decoder_id] / (double)fps[decoder_id];
    spdlog::critical("\t[time];[{}];{};seconds", types_map[decoder_id], time);
    spdlog::critical("\t[original_size];[{}];{};bytes", types_map[decoder_id],
                     original_sizes[decoder_id]);
    spdlog::critical("\t[compressed_size];[{}];{};bytes", types_map[decoder_id],
                     compressed_sizes[decoder_id]);

    spdlog::critical("\t[original_bandwidth];[{}];{};Mbps",
                     types_map[decoder_id],
                     (original_sizes[decoder_id] * 8.0 / 1000000) / (time));
    spdlog::critical("\t[compressed_bandwidth];[{}];{};Mbps",
                     types_map[decoder_id],
                     (compressed_sizes[decoder_id] * 8.0 / 1000000) / (time));

    spdlog::critical("\t[compression ratio];[{}];{};x", types_map[decoder_id],
                     original_sizes[decoder_id] /
                         (float)compressed_sizes[decoder_id]);

    spdlog::critical("\t[latency];[{}];{};ms", types_map[decoder_id],
                     total_latencies[decoder_id] / (float)counts[decoder_id]);

    if (types_map[decoder_id] == 0) {
      spdlog::critical("\t[PSNR];[{}];{}", types_map[decoder_id],
                       psnrs[decoder_id] / counts[decoder_id]);
      cv::Vec<double, 4> mssimV = (mssims[decoder_id] / counts[decoder_id]);
      spdlog::critical("\t[MSSIM];[{}];{};{};{};{}", types_map[decoder_id],
                       mssimV(0), mssimV(1), mssimV(2), mssimV(3));
    } else {
      spdlog::critical(
          "\t[MSE];[{}];{}", types_map[decoder_id],
          mses[decoder_id] /
              (counts[decoder_id] * cols[decoder_id] * rows[decoder_id]));
      spdlog::critical(
          "\t[MSE_4096];[{}];{}", types_map[decoder_id],
          mse_comps[decoder_id] /
              (counts[decoder_id] * cols[decoder_id] * rows[decoder_id]));
    }
  }

  return 0;
}