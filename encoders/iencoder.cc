/**
 * \file iencoder.cc @brief IEncoder factory
 */

#include "iencoder.h"
#include "../utils/logger.h"

#include <ctime>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <thread>

#include <yaml-cpp/yaml.h>
#include <zmq.hpp>

#include "../encoders/libav_encoder.h"
#include "../encoders/null_encoder.h"
#include "../encoders/zdepth_encoder.h"
#include "../readers/video_file_reader.h"
#include "../readers/multi_image_reader.h"

#ifdef SSP_WITH_NVPIPE_SUPPORT
#include "../encoders/nv_encoder.h"
#endif

#ifdef SSP_WITH_KINECT_SUPPORT
#include "../readers/kinect_reader.h"
#include "../utils/kinect_utils.h"
#endif

namespace moetsi::ssp {
    std::unordered_map<FrameType, std::shared_ptr<IEncoder>> IEncoderFactory(const std::string & config, const std::vector<FrameType> &types) {
        std::unordered_map<FrameType, std::shared_ptr<IEncoder>> encoders;
        std::string codec_parameters_file = std::string(config);
        YAML::Node codec_parameters = YAML::LoadFile(codec_parameters_file);
        for (FrameType type : types) {
            try {
                YAML::Node v = codec_parameters["video_encoder"][unsigned(type)];

                std::string encoder_type = v["type"].as<std::string>();
                std::shared_ptr<IEncoder> fe;
                if (encoder_type == "libav")
                    fe = std::shared_ptr<LibAvEncoder>(new LibAvEncoder(v, v["fps"].as<int>()));
                else if (encoder_type == "nvenc") {
        #ifdef SSP_WITH_NVPIPE_SUPPORT
                    fe = std::shared_ptr<NvEncoder>(new NvEncoder(v, v["fps"].as<int>()));
        #else
                    spdlog::error("SSP compiled without \"nvenc\" reader support. Set to "
                                "SSP_WITH_NVPIPE_SUPPORT=ON when configuring with cmake");
                //    return encoders;
                    continue;
        #endif
                } else if (encoder_type == "zdepth")
                    fe = std::shared_ptr<ZDepthEncoder>(new ZDepthEncoder(v, v["fps"].as<int>()));
                else if (encoder_type == "null")
                    fe = std::shared_ptr<NullEncoder>(new NullEncoder(v["fps"].as<int>()));
                else {
                    spdlog::error("Unknown encoder type: \"{}\". Supported types are "
                                "\"libav\", \"nvenc\", \"zdepth\" and \"null\"",
                                encoder_type);
                //    return encoders;
                    continue;
                }
                encoders[type] = fe;
            } catch(std::exception &e) {
                spdlog::error("skipping codec {} : {}", unsigned(type), e.what());
            }
        }

        return encoders;
    }

}