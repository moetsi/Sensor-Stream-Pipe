
/**
 * \file ireader.cc @brief IReader factory
 */
#include "ireader.h"
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
#include "../readers/dummy_body_reader.h"

#ifdef SSP_WITH_NVPIPE_SUPPORT
#include "../encoders/nv_encoder.h"
#endif

#ifdef SSP_WITH_KINECT_SUPPORT
#include "../readers/kinect_reader.h"
#include "../utils/kinect_utils.h"
#endif

#ifdef SSP_WITH_DEPTHAI_SUPPORT
#include "../readers/oakd_xlink_reader.h"
#include "depthai/depthai.hpp"
#endif

namespace moetsi::ssp {

// N.B. this is a copy'n'paste from ssp_server.cc
std::shared_ptr<IReader> IReaderFactory(const std::string & config) {
    std::string codec_parameters_file = std::string(config);
    YAML::Node codec_parameters = YAML::LoadFile(codec_parameters_file);
    YAML::Node general_parameters = codec_parameters["general"];
    SetupLogging(general_parameters);
    std::shared_ptr<IReader> reader;

    std::string reader_type =
        general_parameters["frame_source"]["type"].as<std::string>();
    if (reader_type == "frames") {
      if (general_parameters["frame_source"]["parameters"]["path"].IsSequence())
        reader = std::shared_ptr<MultiImageReader>(new MultiImageReader(
            general_parameters["frame_source"]["parameters"]["path"]
                .as<std::vector<std::string>>()));
      else
        reader = std::shared_ptr<ImageReader>(new ImageReader(
            general_parameters["frame_source"]["parameters"]["path"]
                .as<std::string>()));

    } else if (reader_type == "video") {
      std::string path =
          general_parameters["frame_source"]["parameters"]["path"]
              .as<std::string>();

#if TARGET_OS_IOS
      // Find the corresponding path in the application bundle
      NSString* file_path = [NSString stringWithCString:path.c_str()
                                               encoding:[NSString defaultCStringEncoding]];
      NSString* bundle_path = [[NSBundle mainBundle] pathForResource:file_path
                                                              ofType:nil];
      if (bundle_path != nil)
        path = std::string([bundle_path UTF8String]);
#endif

      if (general_parameters["frame_source"]["parameters"]["streams"]
              .IsDefined()) {
        std::vector<unsigned int> streams =
            general_parameters["frame_source"]["parameters"]["streams"]
                .as<std::vector<unsigned int>>();
        reader = std::shared_ptr<VideoFileReader>(
            new VideoFileReader(path, streams));
      } else {
        reader = std::shared_ptr<VideoFileReader>(new VideoFileReader(path));
      }
      return reader;
    } else if (reader_type == "dummybody") {
        reader = std::make_shared<DummyBodyReader>();
        return reader;
    } else if (reader_type == "kinect") {
#ifdef SSP_WITH_KINECT_SUPPORT
      ExtendedAzureConfig c = BuildKinectConfigFromYAML(
          general_parameters["frame_source"]["parameters"]);
      reader = std::shared_ptr<KinectReader>(new KinectReader(0, c));
#else
      return reader;
#endif
    } else if (reader_type == "iphone") {
#if TARGET_OS_IOS
      reader = std::shared_ptr<iPhoneReader>(new iPhoneReader());
#else
      return reader;
#endif
    } else if (reader_type == "oakd_xlink") {
#ifdef SSP_WITH_DEPTHAI_SUPPORT
      reader = std::shared_ptr<OakdXlinkReader>(new OakdXlinkReader(general_parameters["frame_source"]["parameters"]));
#else
      return reader;
#endif
    } else {
      spdlog::error("Unknown reader type: \"{}\". Supported types are "
                    "\"frames\", \"video\" and \"kinect\"",
                    reader_type);
      return reader;
    }
    return reader;
}

} // namespace moetsi::ssp
