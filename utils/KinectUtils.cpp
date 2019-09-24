//
// Created by amourao on 02/09/19.
//

#include "KinectUtils.h"

ExtendedAzureConfig buildKinectConfigFromYAML(YAML::Node config) {
  ExtendedAzureConfig extendedAzureConfig;
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

  int absoluteExposureValue = 0;
  if (config["absoluteExposureValue"].IsDefined()) {
    absoluteExposureValue = config["absoluteExposureValue"].as<int>();
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

  extendedAzureConfig.device_config = device_config;
  extendedAzureConfig.stream_color = stream_color_video;
  extendedAzureConfig.stream_depth = stream_depth_video;
  extendedAzureConfig.stream_ir = stream_ir_video;
  extendedAzureConfig.absoluteExposureValue = absoluteExposureValue;

  return extendedAzureConfig;
}
