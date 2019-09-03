//
// Created by amourao on 02/09/19.
//

#pragma once


#include <k4a/k4a.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

struct ExtendedAzureConfig {
    k4a_device_configuration_t device_config;
    bool stream_color;
    bool stream_depth;
    bool stream_ir;
    int absoluteExposureValue;
};

ExtendedAzureConfig buildKinectConfigFromYAML(YAML::Node config);
