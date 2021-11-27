/**
 * \file kinect_utils.cc @brief Utils for Kinect RT integration
 */
// Created by amourao on 02/09/19.
#include "kinect_utils.h"

namespace moetsi::ssp {

ExtendedAzureConfig BuildKinectConfigFromYAML(YAML::Node config) {
  ExtendedAzureConfig extended_azure_config;
  k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;

  if (!config["streaming_color_format"].IsDefined()) {
    spdlog::warn("Missing key: \"streaming_color_format\", Using default: "
                 "K4A_IMAGE_FORMAT_COLOR_MJPG");
    device_config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
  } else {
    std::string streaming_color_format =
        config["streaming_color_format"].as<std::string>();
    if (streaming_color_format == "K4A_IMAGE_FORMAT_COLOR_BGRA32") {
      device_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    } else if (streaming_color_format == "K4A_IMAGE_FORMAT_COLOR_MJPG") {
      device_config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
    } else if (streaming_color_format == "K4A_IMAGE_FORMAT_COLOR_YUY2" ||
               streaming_color_format == "K4A_IMAGE_FORMAT_COLOR_NV12") {
      // device_config.color_format = K4A_IMAGE_FORMAT_COLOR_YUY2;
      spdlog::error(
          "\"streaming_color_format\" not supported: {}, Supported values are "
          "K4A_IMAGE_FORMAT_COLOR_BGRA32 and K4A_IMAGE_FORMAT_COLOR_MJPG",
          streaming_color_format);
      throw std::invalid_argument("Not supported: \"streaming_color_format\"");
    } else {
      spdlog::error(
          "Invalid value for \"streaming_color_format\": {}, Supported values "
          "are K4A_IMAGE_FORMAT_COLOR_BGRA32 and K4A_IMAGE_FORMAT_COLOR_MJPG",
          streaming_color_format);
      throw std::invalid_argument(
          "Invalid value for: \"streaming_color_format\"");
    }
  }

  if (!config["streaming_color_resolution"].IsDefined()) {
    spdlog::warn("Missing key: \"streaming_color_resolution\", Using default: "
                 "K4A_COLOR_RESOLUTION_720P");
    device_config.color_resolution = K4A_COLOR_RESOLUTION_720P;
  } else {
    std::string streaming_color_resolution =
        config["streaming_color_resolution"].as<std::string>();
    if (streaming_color_resolution == "K4A_COLOR_RESOLUTION_720P") {
      device_config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    } else if (streaming_color_resolution == "K4A_COLOR_RESOLUTION_1080P") {
      device_config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    } else if (streaming_color_resolution == "K4A_COLOR_RESOLUTION_1440P") {
      device_config.color_resolution = K4A_COLOR_RESOLUTION_1440P;
    } else if (streaming_color_resolution == "K4A_COLOR_RESOLUTION_1536P") {
      device_config.color_resolution = K4A_COLOR_RESOLUTION_1536P;
    } else if (streaming_color_resolution == "K4A_COLOR_RESOLUTION_2160P") {
      device_config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
    } else if (streaming_color_resolution == "K4A_COLOR_RESOLUTION_3072P") {
      device_config.color_resolution = K4A_COLOR_RESOLUTION_3072P;
    } else if (streaming_color_resolution == "K4A_COLOR_RESOLUTION_OFF") {
      device_config.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    } else {
      spdlog::error("Invalid value for \"streaming_color_resolution\": {}, "
                    "Supported values are K4A_COLOR_RESOLUTION_OFF and "
                    "K4A_COLOR_RESOLUTION_720P, K4A_COLOR_RESOLUTION_1080P, "
                    "K4A_COLOR_RESOLUTION_1440P, K4A_COLOR_RESOLUTION_1536P, "
                    "K4A_COLOR_RESOLUTION_2160P and K4A_COLOR_RESOLUTION_3072P",
                    streaming_color_resolution);
      throw std::invalid_argument(
          "Invalid value for: \"streaming_color_resolution\"");
    }
  }

  if (!config["streaming_depth_mode"].IsDefined()) {
    spdlog::warn("Missing key: \"streaming_depth_mode\", Using default: "
                 "K4A_DEPTH_MODE_NFOV_UNBINNED");
    device_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
  } else {
    std::string streaming_depth_mode =
        config["streaming_depth_mode"].as<std::string>();
    if (streaming_depth_mode == "K4A_DEPTH_MODE_PASSIVE_IR") {
      device_config.depth_mode = K4A_DEPTH_MODE_PASSIVE_IR;
    } else if (streaming_depth_mode == "K4A_DEPTH_MODE_WFOV_2X2BINNED") {
      device_config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
    } else if (streaming_depth_mode == "K4A_DEPTH_MODE_WFOV_2X2BINNED") {
      device_config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
    } else if (streaming_depth_mode == "K4A_DEPTH_MODE_NFOV_UNBINNED") {
      device_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    } else if (streaming_depth_mode == "K4A_DEPTH_MODE_NFOV_2X2BINNED") {
      device_config.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;
    } else if (streaming_depth_mode == "K4A_DEPTH_MODE_OFF") {
      device_config.depth_mode = K4A_DEPTH_MODE_OFF;
    } else {
      spdlog::error(
          "Invalid value for \"streaming_depth_mode\": {}, "
          "Supported values are K4A_DEPTH_MODE_OFF and "
          "K4A_DEPTH_MODE_NFOV_2X2BINNED, K4A_DEPTH_MODE_NFOV_UNBINNED, "
          "K4A_DEPTH_MODE_WFOV_2X2BINNED, K4A_DEPTH_MODE_WFOV_UNBINNED, "
          "K4A_DEPTH_MODE_PASSIVE_IR",
          streaming_depth_mode);
      throw std::invalid_argument(
          "Invalid value for: \"streaming_depth_mode\"");
    }
  }

  if (!config["streaming_rate"].IsDefined()) {
    spdlog::warn("Missing key: \"streaming_rate\", Using default: "
                 "K4A_FRAMES_PER_SECOND_30");
    device_config.camera_fps = K4A_FRAMES_PER_SECOND_30;
  } else {
    std::string fps_str = config["streaming_rate"].as<std::string>();
    if (fps_str == "K4A_FRAMES_PER_SECOND_5") {
      device_config.camera_fps = K4A_FRAMES_PER_SECOND_5;
    } else if (fps_str == "K4A_FRAMES_PER_SECOND_15") {
      device_config.camera_fps = K4A_FRAMES_PER_SECOND_15;
    } else if (fps_str == "K4A_FRAMES_PER_SECOND_30") {
      device_config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    } else {
      spdlog::error("Invalid value for \"streaming_rate\": {}, "
                    "Supported values are K4A_FRAMES_PER_SECOND_5, "
                    "K4A_FRAMES_PER_SECOND_15, "
                    "K4A_FRAMES_PER_SECOND_30",
                    fps_str);
      throw std::invalid_argument("Invalid value for: \"streaming_rate\"");
    }
  }

  if (!config["wired_sync_mode"].IsDefined()) {
    spdlog::warn("Missing key: \"wired_sync_mode\", Using default: "
                 "K4A_WIRED_SYNC_MODE_STANDALONE");
    device_config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
  } else {
    std::string wired_sync_mode = config["wired_sync_mode"].as<std::string>();
    if (wired_sync_mode == "K4A_WIRED_SYNC_MODE_MASTER") {
      device_config.wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
    } else if (wired_sync_mode == "K4A_WIRED_SYNC_MODE_SUBORDINATE") {
      device_config.wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
    } else if (wired_sync_mode == "K4A_WIRED_SYNC_MODE_STANDALONE") {
      device_config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
    } else {
      spdlog::error(
          "Invalid value for \"streaming_rate\": {}, "
          "Supported values are K4A_WIRED_SYNC_MODE_MASTER, "
          "K4A_WIRED_SYNC_MODE_SUBORDINATE and K4A_WIRED_SYNC_MODE_STANDALONE",
          wired_sync_mode);
      throw std::invalid_argument("Invalid value for: \"wired_sync_mode\"");
    }
  }

  device_config.depth_delay_off_color_usec = 0;
  if (config["depth_delay_off_color_usec"].IsDefined()) {
    device_config.depth_delay_off_color_usec =
        config["depth_delay_off_color_usec"].as<int>();
  } else {
    spdlog::warn("Using default depth_delay_off_color_usec = 0");
  }

  device_config.subordinate_delay_off_master_usec = 0;
  if (config["subordinate_delay_off_master_usec"].IsDefined()) {
    device_config.subordinate_delay_off_master_usec =
        config["subordinate_delay_off_master_usec"].as<int>();
  } else {
    spdlog::warn("Using default subordinate_delay_off_master_usec = 0");
  }

  int absolute_exposure_value = 0;
  if (config["absoluteExposureValue"].IsDefined()) {
    absolute_exposure_value = config["absoluteExposureValue"].as<int>();
  } else {
    ;
    spdlog::warn("Using default absoluteExposureValue = 0");
  }

  bool stream_color_video = true;

  if (!config["stream_color_video"].IsDefined()) {
    spdlog::warn("Missing key: \"stream_color_video\", Using default: "
                 "true");
  } else {
    stream_color_video = config["stream_color_video"].as<bool>();
    if (!stream_color_video) {
      device_config.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    }
  }

  bool stream_depth_video = true;
  if (!config["stream_depth_video"].IsDefined()) {
    spdlog::warn("Missing key: \"stream_depth_video\", Using default: "
                 "true");
  } else {
    stream_depth_video = config["stream_depth_video"].as<bool>();
    if (!stream_depth_video) {
      device_config.depth_mode = K4A_DEPTH_MODE_PASSIVE_IR;
    }
  }

  bool stream_ir_video = false;
  if (!config["stream_ir_video"].IsDefined()) {
    spdlog::warn("Missing key: \"stream_ir_video\", Using default: "
                 "false");
  } else {
    stream_ir_video = config["stream_ir_video"].as<bool>();
  }

  if (!stream_ir_video && !stream_depth_video) {
    device_config.depth_mode = K4A_DEPTH_MODE_OFF;
  }

  extended_azure_config.device_config = device_config;
  extended_azure_config.stream_color = stream_color_video;
  extended_azure_config.stream_depth = stream_depth_video;
  extended_azure_config.stream_ir = stream_ir_video;
  extended_azure_config.absolute_exposure_value = absolute_exposure_value;

  return extended_azure_config;
}

void FrameStructToK4A(std::vector<FrameStruct> &fs,
                      k4a::capture &sensor_capture,
                      std::unordered_map<std::string, std::shared_ptr<IDecoder>> &decoders) {
  for (auto &f : fs) {
    std::string decoder_id = f.stream_id + std::to_string(f.sensor_id);

    if (decoders.find(decoder_id) == decoders.end()) {
      CodecParamsStruct data = f.codec_data;

      if (data.type == CodecParamsType::CodecParamsTypeAv) {
        std::shared_ptr<LibAvDecoder> fd = std::shared_ptr<LibAvDecoder>(new LibAvDecoder());
        fd->Init(getParams(f));
        decoders[decoder_id] = fd;

      } else if (data.type == CodecParamsType::CodecParamsTypeNvPipe) {
#ifdef SSP_WITH_NVPIPE_SUPPORT
        std::shared_ptr<NvDecoder> fd = std::shared_ptr<NvDecoder>(new NvDecoder());
        fd->Init(data.data);
        decoders[decoder_id] = fd;
#else
        spdlog::error("SSP compiled without \"nvenc\" reader support. Set to "
                    "SSP_WITH_NVPIPE_SUPPORT=ON when configuring with cmake");
      exit(1);
#endif

      } else if (data.type == CodecParamsType::CodecParamsTypeZDepth) {
        std::shared_ptr<ZDepthDecoder> fd = std::shared_ptr<ZDepthDecoder>(new ZDepthDecoder());
        fd->Init(data.data);
        decoders[decoder_id] = fd;
      }
    }

    cv::Mat img;

    if (f.frame_data_type == FrameDataType::FrameDataTypeImageFrame) {
      img = cv::imdecode(f.frame, CV_LOAD_IMAGE_UNCHANGED);

    } else if (f.frame_data_type == FrameDataType::FrameDataTypeRawRGBA) {
      int rows, cols;
      memcpy(&cols, &f.frame[0], sizeof(int));
      memcpy(&rows, &f.frame[4], sizeof(int));
      img =
          cv::Mat(rows, cols, CV_8UC4, (void *)&f.frame[8], cv::Mat::AUTO_STEP);

    } else if (f.frame_data_type == FrameDataType::FrameDataTypeGRAY16LE) {
      int rows, cols;
      memcpy(&cols, &f.frame[0], sizeof(int));
      memcpy(&rows, &f.frame[4], sizeof(int));
      img = cv::Mat(rows, cols, CV_16UC1, (void *)&f.frame[8],
                    cv::Mat::AUTO_STEP);

    } else if (f.frame_data_type == FrameDataType::FrameDataTypeLibavPackets) {

      std::shared_ptr<IDecoder> decoder;

      std::string decoder_id = f.stream_id + std::to_string(f.sensor_id);

      if (decoders.find(decoder_id) == decoders.end()) {
        CodecParamsStruct data = f.codec_data;

        if (data.type == CodecParamsType::CodecParamsTypeAv) {
          std::shared_ptr<LibAvDecoder> fd = std::shared_ptr<LibAvDecoder>(new LibAvDecoder());
          fd->Init(getParams(f));
          decoders[decoder_id] = fd;

        } else if (data.type == CodecParamsType::CodecParamsTypeNvPipe) {
#ifdef SSP_WITH_NVPIPE_SUPPORT
          std::shared_ptr<NvDecoder> fd = std::shared_ptr<NvDecoder>(new NvDecoder());
          fd->Init(data.data);
          decoders[decoder_id] = fd;
#else
          spdlog::error("SSP compiled without \"nvenc\" reader support. Set to "
                    "SSP_WITH_NVPIPE_SUPPORT=ON when configuring with cmake");
      exit(1);
#endif

        } else if (data.type == CodecParamsType::CodecParamsTypeZDepth) {
          std::shared_ptr<ZDepthDecoder> fd = std::shared_ptr<ZDepthDecoder>(new ZDepthDecoder());
          fd->Init(data.data);
          decoders[decoder_id] = fd;
        }
      }

      decoder = decoders[decoder_id];

      img = decoder->Decode(f);

      f.frame.clear();
    }


    if (f.frame_type == FrameType::FrameTypeColor) {
      if (img.channels() == 3)
        cv::cvtColor(img, img, cv::COLOR_BGR2BGRA);
      k4a::image color = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                                            img.cols, img.rows, 4 * img.cols);
      memcpy(color.get_buffer(), img.datastart, img.total() * img.elemSize());
      sensor_capture.set_color_image(color);

    } else if (f.frame_type == FrameType::FrameTypeDepth) {
      k4a::image depth = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, img.cols,
                                            img.rows, 2 * img.cols);
      memcpy(depth.get_buffer(), img.datastart, img.total() * img.elemSize());
      sensor_capture.set_depth_image(depth);

    } else if (f.frame_type == FrameType::FrameTypeIR) {
      k4a::image ir = k4a::image::create(K4A_IMAGE_FORMAT_IR16, img.cols,
                                         img.rows, 2 * img.cols);
      memcpy(ir.get_buffer(), img.datastart, img.total() * img.elemSize());
      sensor_capture.set_ir_image(ir);
    }
  }
}

} // namespace moetsi::ssp
