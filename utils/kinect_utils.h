/**
 * \file kinect_utils.h @brief Utils for Kinect RT integration
 */
// Created by amourao on 02/09/19.
#pragma once
#include <iostream>

#undef min
#undef max
#include <k4a/k4a.hpp>

#if (SSP_WITH_K4A_BODYTRACK)
#include <k4abt.hpp>
#endif

#include <yaml-cpp/yaml.h>

#include "../decoders/idecoder.h"
#include "../decoders/libav_decoder.h"
#include "../structs/frame_struct.h"
#include "logger.h"

namespace moetsi::ssp {

/**
 * @brief Azure Kinect configuration
 */
struct ExtendedAzureConfig {
  /**
   * Device configuration
   */
  k4a_device_configuration_t device_config;
  /**
   * If true, stream color frames
   */
  bool stream_color;
  /**
   * If true, stream depth frames
   */
  bool stream_depth;
  /**
   * If true, stream infrared frames
   */
  bool stream_ir;
  /**
   * Absolute exposure value
   */
  int absolute_exposure_value;
};

/**
 * @brief Build Kinect configuration from YAML configuration
 * \param config yaml confirguration
 * \return Azure Kinect configuration
 */
ExtendedAzureConfig BuildKinectConfigFromYAML(YAML::Node config);

/**
 * @brief Transform frame structure to K4A format
 * Update decoder dictionary
 * \param f source frame structure
 * \param sensor_capture destination "capture" structure
 * \param decoders decoders map - updated
 */
void FrameStructToK4A(std::vector<FrameStruct> &f, k4a::capture &sensor_capture,
                      std::unordered_map<std::string, std::shared_ptr<IDecoder >> &decoders);

} // namespace moetsi::ssp
