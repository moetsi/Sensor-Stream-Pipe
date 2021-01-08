//
// Created by amourao on 02/09/19.
//

#pragma once

#include <iostream>
#include <k4a/k4a.hpp>

#if (SSP_WITH_K4A_BODYTRACK)
#include <k4abt.hpp>
#endif

#include <yaml-cpp/yaml.h>

#include "../decoders/idecoder.h"
#include "../decoders/libav_decoder.h"
#include "../structs/frame_struct.hpp"
#include "logger.h"

struct ExtendedAzureConfig {
  k4a_device_configuration_t device_config;
  bool stream_color;
  bool stream_depth;
  bool stream_ir;
  int absolute_exposure_value;
};

ExtendedAzureConfig BuildKinectConfigFromYAML(YAML::Node config);

void FrameStructToK4A(std::vector<FrameStruct> &f, k4a::capture &sensor_capture,
                      std::unordered_map<std::string, std::shared_ptr<IDecoder >> &decoders);