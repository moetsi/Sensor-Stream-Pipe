
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
#include "../readers/oakd_device_reader.h"
#include "depthai/depthai.hpp"
#endif

namespace moetsi::ssp {

/**
 * @brief Magic string interpolation
 * \param env environment
 * \param str input
 */
std::string StringInterpolation(const std::map<std::string, std::string> &env, const std::string &s) {
    bool had_subst = false;
    auto rv = s;
    do {
        had_subst = false;
        // find @{
        std::vector<unsigned> begins;
        bool last_is_at = false;
        std::stringstream ss;
        unsigned cursor = 0;

        auto advance_cursor = [&](unsigned nc) {
            ss << rv.substr(cursor, nc - cursor);
            //std::cerr << "advanced: " << ss.str() << " " << cursor << " " << nc << std::endl << std::flush;
            cursor = nc; 
        };

        for (unsigned i=0; i< rv.size(); ++i) {
            if (rv[i] == '{' && last_is_at) {
                begins.push_back(i);
                // std::cerr << "@{ ... " << std::to_string(i) << std::endl << std::flush;
            }
            if (rv[i] == '}') {
                if (begins.size() > 0) {
                    auto x = begins.back();
                    begins.pop_back();

                    auto key = rv.substr(x+1, i-x-1);
                    // std::cerr << "key = " << key << " " << i << " " << x << std::endl << std::flush;
                    advance_cursor(std::max(cursor, x-1));

                    auto e = std::getenv(key.c_str());
                    if (e != nullptr) {
                        auto ee = std::string(e);
                        //std::cerr << "env: " << ee << std::endl << std::flush;
                        if (ee.size() > 0) {
                            ss << ee;
                            had_subst = true;
                            cursor = i+1;
                        }
                    } else {
                        auto it = env.find(key);
                        if (it != env.end()) {
                            ss << it->second;
                            had_subst = true;
                            cursor = i+1;
                        } else {

                        }
                    }
                }
            }

            last_is_at = rv[i] == '@';
        }
        advance_cursor(rv.size()+1);
        rv = ss.str();
    } while(had_subst);

    //std::cerr << "string interpolation = " << rv << std::endl << std::flush;
    return rv;
}

std::string StringInterpolation(const std::string &s) {
    std::map<std::string, std::string> env;
    return StringInterpolation(env, s);
}

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
    } else if (reader_type == "oakd_device") {
#ifdef SSP_WITH_DEPTHAI_SUPPORT
      reader = std::shared_ptr<OakdDeviceReader>(new OakdDeviceReader(general_parameters["frame_source"]["parameters"]));
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
