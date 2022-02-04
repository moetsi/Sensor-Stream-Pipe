/**
 * \file ssp_client_opencv.cc @brief OpenCV based ssp client client
 */
// Created by amourao on 26-06-2019.

#include <chrono>
#include <iostream>
#include <thread>

#ifdef _WIN32
#include <io.h>
#define SSP_EXPORT __declspec(dllexport)
#else
#include <unistd.h>
#define SSP_EXPORT
#endif 

#include <opencv2/imgproc.hpp>
#include <zmq.hpp>

extern "C" {
#ifdef FFMPEG_AS_FRAMEWORK
#include <FFmpeg/avcodec.h>
#include <FFmpeg/avformat.h>
#include <FFmpeg/avutil.h>
#include <FFmpeg/pixdesc.h>
#include <FFmpeg/swscale.h>
#else
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
#endif
}

#include "../utils/logger.h"

#include "../readers/network_reader.h"
#include "../utils/video_utils.h"
#include "../utils/image_converter.h"

// imshow not available on iOS/iPhone Simulator
#if __APPLE__
  #include <TargetConditionals.h>
  #if TARGET_OS_MAC
    #define HAS_IMSHOW 1
  #else
    #define HAS_IMSHOW 0
  #endif
#else
  #define HAS_IMSHOW 1
#endif

using namespace moetsi::ssp;



extern "C" SSP_EXPORT int ssp_client_opencv(int port)
{
  av_log_set_level(AV_LOG_QUIET);

  try {
    NetworkReader reader(port);

    reader.init();

    std::unordered_map<std::string, std::shared_ptr<IDecoder>> decoders;

    bool imgChanged = false;
    int c = 0;

    // For marking 2D pose locations
    float confidenceThreshold = 0;
    cv::Scalar red = cv::Scalar( 0, 0, 255 );
    cv::Scalar blue = cv::Scalar( 255, 0, 0 );
    int markerRadius = 5;

    while (reader.HasNextFrame()) {
      reader.NextFrame();
      std::vector<FrameStruct> f_list = reader.GetCurrentFrame();

      bool detectedBody = false;
      coco_human_t bodyStruct;

      for (FrameStruct f : f_list) {
        std::string decoder_id = f.stream_id + std::to_string(f.sensor_id);

        if (f.frame_type == FrameType::FrameTypeHumanPose)
        {
          memcpy(&bodyStruct, &f.frame[4], sizeof(coco_human_t));
          detectedBody = true;
        }

        cv::Mat img;
        imgChanged = FrameStructToMat(f, img, decoders);

        if (imgChanged && !img.empty()) {

          // Manipulate image to show as a color map
          if (f.frame_type == FrameType::FrameTypeDepth) {
            if (img.type() == CV_16U && f.frame_data_type != FrameDataType::FrameDataTypeDepthAIStereoDepth) {
              // Compress images to show up on a 255 valued color map
              img *= (MAX_DEPTH_VALUE_8_BITS / (float)MAX_DEPTH_VALUE_12_BITS);
              cv::normalize(img, img, 255, 0, cv::NORM_MINMAX);
              cv::Mat imgOut;
              img.convertTo(imgOut, CV_8U);
              cv::applyColorMap(imgOut, img, cv::COLORMAP_JET);

            } else if (img.type() == CV_32F) {
              // Normalize image to 0;255
              cv::normalize(img, img, 255, 0, cv::NORM_MINMAX);
              cv::Mat imgOut;
              img.convertTo(imgOut, CV_8U);
              cv::applyColorMap(imgOut, img, cv::COLORMAP_JET);

            } else if (img.type() == CV_16U && f.frame_data_type == FrameDataType::FrameDataTypeDepthAIStereoDepth) {
              // uint8_t *ptr = (uint8_t*) img.data;
              // for (unsigned i=0; i< img.rows; ++i) {
              //   for (unsigned j=0; j < img.cols; ++j) {
              //     auto v = ptr[i*img.cols + j];
              //     // 400P: 441.25
              //     // 800P: 882.5
              //     float depthcm = 441.25 * 7.5 / float(v);
              //     // adjust to 8 bits
              //     float v2 = depthcm; // 1024 cm ~10 m
              //      ptr[i*img.cols + j] = uint8_t(v2);
              //   }
              // }
              // cv::Mat imgOut;
              // img.convertTo(imgOut, CV_8U);
              // cv::applyColorMap(imgOut, img, cv::COLORMAP_JET);
              
              cv::Mat depthFrameColor;
              cv::normalize(img, depthFrameColor, 255, 0, cv::NORM_INF, CV_8UC1);
              cv::equalizeHist(depthFrameColor, depthFrameColor);
              cv::applyColorMap(depthFrameColor, depthFrameColor, cv::COLORMAP_HOT);    
              img = depthFrameColor; 
            }

          } else if (f.frame_type == FrameType::FrameTypeIR) {

            double max = 1024;
            img *= (MAX_DEPTH_VALUE_8_BITS / (float)max);
            img.convertTo(img, CV_8U);
          } else if (f.frame_type == FrameType::FrameTypeConfidence) {
            cv::Mat imgOut;
            img *= 127; // iOS confidence is 0:low, 1:medium, 2:high
          } else if (f.frame_type == FrameType::FrameTypeColor && detectedBody)
          {

            cv::imwrite("img-" + std::to_string(c) + ".jpg", img);

            //// build export struct [2d]
            {
              std::stringstream ss;
              ss << "[" << std::endl;
              ss << "[" 
                << bodyStruct.neck_2d_x << ","
                << bodyStruct.neck_2d_y << ","
                << bodyStruct.neck_2d_depth << ","
                << bodyStruct.neck_2d_conf
                << "]," << std::endl;
              ss << "["
                << bodyStruct.nose_2d_x << ","
                << bodyStruct.nose_2d_y << ","
                << bodyStruct.nose_2d_depth << ","
                << bodyStruct.nose_2d_conf
                << "]," << std::endl;
              ss << "["
                << bodyStruct.pelvis_2d_x << ","
                << bodyStruct.pelvis_2d_y << ","
                << bodyStruct.pelvis_2d_depth << ","
                << bodyStruct.pelvis_2d_conf
                  << "]," << std::endl;
              ss << "["
                << bodyStruct.shoulder_left_2d_x << ","
                << bodyStruct.shoulder_left_2d_y << ","
                << bodyStruct.shoulder_left_2d_depth << ","
                << bodyStruct.shoulder_left_2d_conf
                  << "]," << std::endl;
              ss << "["
                << bodyStruct.elbow_left_2d_x << ","
                << bodyStruct.elbow_left_2d_y << ","
                << bodyStruct.elbow_left_2d_depth << ","
                << bodyStruct.elbow_left_2d_conf
                  << "]," << std::endl;
              ss << "["
                << bodyStruct.wrist_left_2d_x << ","
                << bodyStruct.wrist_left_2d_y << ","
                << bodyStruct.wrist_left_2d_depth << ","
                << bodyStruct.wrist_left_2d_conf
                  << "]," << std::endl;
              ss << "["
                << bodyStruct.hip_left_2d_x << ","
                << bodyStruct.hip_left_2d_y << ","
                << bodyStruct.hip_left_2d_depth << ","
                << bodyStruct.hip_left_2d_conf
                  << "]," << std::endl;
              ss << "["
                  << bodyStruct.knee_left_2d_x << ","
                  << bodyStruct.knee_left_2d_y << ","
                  << bodyStruct.knee_left_2d_depth << ","
                  << bodyStruct.knee_left_2d_conf
                  << "]," << std::endl;
              ss << "["
                  << bodyStruct.ankle_left_2d_x << ","
                  << bodyStruct.ankle_left_2d_y << ","
                  << bodyStruct.ankle_left_2d_depth << ","
                  << bodyStruct.ankle_left_2d_conf
                  << "]," << std::endl;
              ss << "["
                  << bodyStruct.shoulder_right_2d_x << ","
                  << bodyStruct.shoulder_right_2d_y << ","
                  << bodyStruct.shoulder_right_2d_depth << ","
                  << bodyStruct.shoulder_right_2d_conf
                  << "]," << std::endl;
              ss << "["
                  << bodyStruct.elbow_right_2d_x << ","
                  << bodyStruct.elbow_right_2d_y << ","
                  << bodyStruct.elbow_right_2d_depth << ","
                  << bodyStruct.elbow_right_2d_conf
                  << "]," << std::endl;
              ss << "["
                  << bodyStruct.wrist_right_2d_x << ","
                  << bodyStruct.wrist_right_2d_y << ","
                  << bodyStruct.wrist_right_2d_depth << ","
                  << bodyStruct.wrist_right_2d_conf
                  << "]," << std::endl;
              ss << "["
                  << bodyStruct.hip_right_2d_x << ","
                  << bodyStruct.hip_right_2d_y << ","
                  << bodyStruct.hip_right_2d_depth << ","
                  << bodyStruct.hip_right_2d_conf
                  << "]," << std::endl;
              ss << "["
                  << bodyStruct.knee_right_2d_x << ","
                  << bodyStruct.knee_right_2d_y << ","
                  << bodyStruct.knee_right_2d_depth << ","
                  << bodyStruct.knee_right_2d_conf 
                  << "]," << std::endl;
              ss << "["
                  << bodyStruct.ankle_right_2d_x << ","
                  << bodyStruct.ankle_right_2d_y << ","
                  << bodyStruct.ankle_right_2d_depth << "," 
                  << bodyStruct.ankle_right_2d_conf 
                  << "]," << std::endl;
              ss << "["
                  << bodyStruct.eye_left_2d_x << ","
                  << bodyStruct.eye_left_2d_y << ","
                  << bodyStruct.eye_left_2d_depth << ","
                  << bodyStruct.eye_left_2d_conf
                  << "]," << std::endl;
              ss << "["
                  << bodyStruct.ear_left_2d_x << ","
                  << bodyStruct.ear_left_2d_y << ","
                  << bodyStruct.ear_left_2d_depth << ","
                  << bodyStruct.ear_left_2d_conf
                  << "]," << std::endl;
              ss << "["
                  << bodyStruct.eye_right_2d_x << ","
                  << bodyStruct.eye_right_2d_y << ","
                  << bodyStruct.eye_right_2d_depth << ","
                  << bodyStruct.eye_right_2d_conf
                  << "]," << std::endl;
              ss << "["
                  << bodyStruct.ear_right_2d_x << ","
                  << bodyStruct.ear_right_2d_y << ","
                  << bodyStruct.ear_right_2d_depth << ","
                  << bodyStruct.ear_right_2d_conf
                  << "]" << std::endl;
              ss << "]" << std::endl;
               std::ofstream w("2d-pose-" + std::to_string(c) + ".txt");
              w << ss.str() << std::endl << std::flush;
              w.close();
            }

            //// same, [3d]
            {
              std::stringstream ss;
              ss << "[" << std::endl;
              ss << "["
                  << bodyStruct.neck_x << ","
                  << bodyStruct.neck_y << ","
                  << bodyStruct.neck_z << ","
                  << bodyStruct.neck_conf
                  << "]," << std::endl << std::flush;
              ss << "["
                  << bodyStruct.nose_x << ","
                  << bodyStruct.nose_y << ","
                  << bodyStruct.nose_z << ","
                  << bodyStruct.nose_conf
                  << "]," << std::endl << std::flush;
              ss << "["
                  << bodyStruct.pelvis_x << ","
                  << bodyStruct.pelvis_y << ","
                  << bodyStruct.pelvis_z << ","
                  << bodyStruct.pelvis_conf
                  << "]," << std::endl << std::flush;
              ss << "["
                  << bodyStruct.shoulder_left_x << ","
                  << bodyStruct.shoulder_left_y << ","
                  << bodyStruct.shoulder_left_z << ","
                  << bodyStruct.shoulder_left_conf
                  << "]," << std::endl << std::flush;
              ss << "["
                  << bodyStruct.elbow_left_x << ","
                  << bodyStruct.elbow_left_y << ","
                  << bodyStruct.elbow_left_z << ","
                  << bodyStruct.elbow_left_conf
                  << "]," << std::endl << std::flush;
              ss << "["
                  << bodyStruct.wrist_left_x << ","
                  << bodyStruct.wrist_left_y << ","
                  << bodyStruct.wrist_left_z << ","
                  << bodyStruct.wrist_left_conf
                  << "]," << std::endl << std::flush;
              ss << "["
                  << bodyStruct.hip_left_x << ","
                  << bodyStruct.hip_left_y << ","
                  << bodyStruct.hip_left_z << ","
                  << bodyStruct.hip_left_conf
                  << "]," << std::endl << std::flush;
              ss << "["
                  << bodyStruct.knee_left_x << ","
                  << bodyStruct.knee_left_y << ","
                  << bodyStruct.knee_left_z << ","
                  << bodyStruct.knee_left_conf
                  << "]," << std::endl << std::flush;
              ss << "["
                  << bodyStruct.ankle_left_x << ","
                  << bodyStruct.ankle_left_y << ","
                  << bodyStruct.ankle_left_z << ","
                  << bodyStruct.ankle_left_conf
                  << "]," << std::endl << std::flush;
              ss << "["
                  << bodyStruct.shoulder_right_x << ","
                  << bodyStruct.shoulder_right_y << ","
                  << bodyStruct.shoulder_right_z << ","
                  << bodyStruct.shoulder_right_conf
                  << "]," << std::endl << std::flush;
              ss << "["
                  << bodyStruct.elbow_right_x << ","
                  << bodyStruct.elbow_right_y << ","
                  << bodyStruct.elbow_right_z << ","
                  << bodyStruct.elbow_right_conf 
                  << "]," << std::endl << std::flush;
              ss << "["
                  << bodyStruct.wrist_right_x << ","
                  << bodyStruct.wrist_right_y << ","
                  << bodyStruct.wrist_right_z << ","
                  << bodyStruct.wrist_right_conf
                  << "]," << std::endl << std::flush;
              ss << "["
                  << bodyStruct.hip_right_x << ","
                  << bodyStruct.hip_right_y << ","
                  << bodyStruct.hip_right_z << ","
                  << bodyStruct.hip_right_conf
                  << "]," << std::endl << std::flush;                  
              ss << "["
                  << bodyStruct.knee_right_x << ","
                  << bodyStruct.knee_right_y << ","
                  << bodyStruct.knee_right_z << ","
                  << bodyStruct.knee_right_conf
                  << "]," << std::endl << std::flush;                    
              ss << "["
                  << bodyStruct.ankle_right_x << ","
                  << bodyStruct.ankle_right_y << ","
                  << bodyStruct.ankle_right_z << ","
                  << bodyStruct.ankle_right_conf
                  << "]," << std::endl << std::flush;                   
              ss << "["
                  << bodyStruct.eye_left_x << ","
                  << bodyStruct.eye_left_y << ","
                  << bodyStruct.eye_left_z << ","
                  << bodyStruct.eye_left_conf 
                  << "]," << std::endl << std::flush; 
              ss << "["
                  << bodyStruct.ear_left_x << ","
                  << bodyStruct.ear_left_y << ","
                  << bodyStruct.ear_left_z << ","
                  << bodyStruct.ear_left_conf
                  << "]," << std::endl << std::flush; 
              ss << "["
                  << bodyStruct.eye_right_x << ","
                  << bodyStruct.eye_right_y << ","
                  << bodyStruct.eye_right_z << ","
                  << bodyStruct.eye_right_conf
                  << "]," << std::endl << std::flush; 
              ss << "["
                  << bodyStruct.ear_right_x << ","
                  << bodyStruct.ear_right_y << ","
                  << bodyStruct.ear_right_z << ","
                  << bodyStruct.ear_right_conf
                  << "]," << std::endl << std::flush; 
              ss << "]" << std::endl;
              std::ofstream w("3d-pose-" + std::to_string(c) + ".txt");
              w << ss.str() << std::endl << std::flush;
              w.close();
            }
            /////

            auto showDistance = [&](const cv::Point &p, cv::Scalar s, float d) {
              auto w = d / 4000;
              auto p2 = cv::Point(p.x + cos(w)*100, p.y + sin(w)*100);
              line(img, p, p2, s, 5);
            };

            {
              // project 3d points 
              float fx = 984.344 / 0.84381;
              auto project = [fx](float x, float y, float z) {
                auto y1 = (fx * x / z) * 0.84381 + 1280/2; 
                auto y2 = (-fx * y / z) * 0.84381 + 720/2;
                return std::make_tuple(y1, y2);
              };
              auto dist = [fx](float x, float y, float z) {
                return std::sqrt(x*x  + y*y + z*z / 0.84381 / 0.84381) * 1000.0;
              };

              if (bodyStruct.neck_x != float(-1)) {
                auto xy = project(bodyStruct.neck_y, bodyStruct.neck_z, -bodyStruct.neck_x);
                circle( img, cv::Point(std::get<0>(xy), std::get<1>(xy)), markerRadius*2, blue, cv::FILLED, cv::LINE_8 );
                std::cerr << "neck: " << std::get<0>(xy) << " " << std::get<1>(xy) << std::endl << std::flush;
              }
              if (bodyStruct.nose_x != float(-1)) {
                auto xy = project(bodyStruct.nose_y, bodyStruct.nose_z, -bodyStruct.nose_x);
                auto d = dist(bodyStruct.nose_y, bodyStruct.nose_z, -bodyStruct.nose_x);
                circle( img, cv::Point(std::get<0>(xy), std::get<1>(xy)), markerRadius*2, blue, cv::FILLED, cv::LINE_8 );
                showDistance(cv::Point(std::get<0>(xy), std::get<1>(xy)), blue, d);
                showDistance(cv::Point(std::get<0>(xy), std::get<1>(xy)), red, bodyStruct.nose_2d_depth);
                std::cerr << "nose: " << std::get<0>(xy) << " " << std::get<1>(xy) << std::endl << std::flush;
                std::cerr << d << " vs. depth = " << bodyStruct.nose_2d_depth << std::endl << std::flush;
              }
              if (bodyStruct.shoulder_left_x != float(-1)) {
                auto xy = project(bodyStruct.shoulder_left_y, bodyStruct.shoulder_left_z, -bodyStruct.shoulder_left_x);
                circle( img, cv::Point(std::get<0>(xy), std::get<1>(xy)), markerRadius*2, blue, cv::FILLED, cv::LINE_8 );
                std::cerr << "left shoulder: " << std::get<0>(xy) << " " << std::get<1>(xy) << std::endl << std::flush;
              }              
              if (bodyStruct.elbow_left_x != float(-1)) {
                auto xy = project(bodyStruct.elbow_left_y, bodyStruct.elbow_left_z, -bodyStruct.elbow_left_x);
                circle( img, cv::Point(std::get<0>(xy), std::get<1>(xy)), markerRadius*2, blue, cv::FILLED, cv::LINE_8 );
                std::cerr << "left elbow: " << std::get<0>(xy) << " " << std::get<1>(xy) << std::endl << std::flush;
              }   
              if (bodyStruct.wrist_left_x != float(-1)) {
                auto xy = project(bodyStruct.wrist_left_y, bodyStruct.wrist_left_z, -bodyStruct.wrist_left_x);
                circle( img, cv::Point(std::get<0>(xy), std::get<1>(xy)), markerRadius*2, blue, cv::FILLED, cv::LINE_8 );
                std::cerr << "left wrist: " << std::get<0>(xy) << " " << std::get<1>(xy) << std::endl << std::flush;
              } 
              if (bodyStruct.hip_left_x != float(-1)) {
                auto xy = project(bodyStruct.hip_left_y, bodyStruct.hip_left_z, -bodyStruct.hip_left_x);
                circle( img, cv::Point(std::get<0>(xy), std::get<1>(xy)), markerRadius*2, blue, cv::FILLED, cv::LINE_8 );
                std::cerr << "left hip: " << std::get<0>(xy) << " " << std::get<1>(xy) << std::endl << std::flush;
              } 
              if (bodyStruct.knee_left_x != float(-1)) {
                auto xy = project(bodyStruct.knee_left_y, bodyStruct.knee_left_z, -bodyStruct.knee_left_x);
                circle( img, cv::Point(std::get<0>(xy), std::get<1>(xy)), markerRadius*2, blue, cv::FILLED, cv::LINE_8 );
                std::cerr << "left knee: " << std::get<0>(xy) << " " << std::get<1>(xy) << std::endl << std::flush;
              } 
              if (bodyStruct.ankle_left_x != float(-1)) {
                auto xy = project(bodyStruct.ankle_left_y, bodyStruct.ankle_left_z, -bodyStruct.ankle_left_x);
                auto d = dist(bodyStruct.ankle_left_y, bodyStruct.ankle_left_z, -bodyStruct.ankle_left_x);
                circle( img, cv::Point(std::get<0>(xy), std::get<1>(xy)), markerRadius*2, blue, cv::FILLED, cv::LINE_8 );
                showDistance(cv::Point(std::get<0>(xy), std::get<1>(xy)), blue, d);
                showDistance(cv::Point(std::get<0>(xy), std::get<1>(xy)), red, bodyStruct.ankle_left_2d_depth);
                std::cerr << "left ankle: " << std::get<0>(xy) << " " << std::get<1>(xy) << std::endl << std::flush;
                std::cerr << d << " vs. depth = " << bodyStruct.ankle_left_2d_depth << std::endl << std::flush;
              }  
              if (bodyStruct.shoulder_right_x != float(-1)) {
                auto xy = project(bodyStruct.shoulder_right_y, bodyStruct.shoulder_right_z, -bodyStruct.shoulder_right_x);
                circle( img, cv::Point(std::get<0>(xy), std::get<1>(xy)), markerRadius*2, blue, cv::FILLED, cv::LINE_8 );
                std::cerr << "right shoulder: " << std::get<0>(xy) << " " << std::get<1>(xy) << std::endl << std::flush;
              }              
              if (bodyStruct.elbow_right_x != float(-1)) {
                auto xy = project(bodyStruct.elbow_right_y, bodyStruct.elbow_right_z, -bodyStruct.elbow_right_x);
                circle( img, cv::Point(std::get<0>(xy), std::get<1>(xy)), markerRadius*2, blue, cv::FILLED, cv::LINE_8 );
                std::cerr << "right elbow: " << std::get<0>(xy) << " " << std::get<1>(xy) << std::endl << std::flush;
              }   
              if (bodyStruct.wrist_right_x != float(-1)) {
                auto xy = project(bodyStruct.wrist_right_y, bodyStruct.wrist_right_z, -bodyStruct.wrist_right_x);
                circle( img, cv::Point(std::get<0>(xy), std::get<1>(xy)), markerRadius*2, blue, cv::FILLED, cv::LINE_8 );
                std::cerr << "right wrist: " << std::get<0>(xy) << " " << std::get<1>(xy) << std::endl << std::flush;
              } 
              if (bodyStruct.hip_right_x != float(-1)) {
                auto xy = project(bodyStruct.hip_right_y, bodyStruct.hip_right_z, -bodyStruct.hip_right_x);
                circle( img, cv::Point(std::get<0>(xy), std::get<1>(xy)), markerRadius*2, blue, cv::FILLED, cv::LINE_8 );
                std::cerr << "right hip: " << std::get<0>(xy) << " " << std::get<1>(xy) << std::endl << std::flush;
              } 
              if (bodyStruct.knee_right_x != float(-1)) {
                auto xy = project(bodyStruct.knee_right_y, bodyStruct.knee_right_z, -bodyStruct.knee_right_x);
                circle( img, cv::Point(std::get<0>(xy), std::get<1>(xy)), markerRadius*2, blue, cv::FILLED, cv::LINE_8 );
                std::cerr << "right knee: " << std::get<0>(xy) << " " << std::get<1>(xy) << std::endl << std::flush;
              } 
              if (bodyStruct.ankle_right_x != float(-1)) {
                auto xy = project(bodyStruct.ankle_right_y, bodyStruct.ankle_right_z, -bodyStruct.ankle_right_x);
                auto d = dist(bodyStruct.ankle_right_y, bodyStruct.ankle_right_z, -bodyStruct.ankle_right_x);
                circle( img, cv::Point(std::get<0>(xy), std::get<1>(xy)), markerRadius*2, blue, cv::FILLED, cv::LINE_8 );
                showDistance(cv::Point(std::get<0>(xy), std::get<1>(xy)), blue, d);
                showDistance(cv::Point(std::get<0>(xy), std::get<1>(xy)), red, bodyStruct.ankle_right_2d_depth);
                std::cerr << "left ankle: " << std::get<0>(xy) << " " << std::get<1>(xy) << std::endl << std::flush;
                std::cerr << d << " vs. depth = " << bodyStruct.ankle_right_2d_depth << std::endl << std::flush;
              } 
              if (bodyStruct.eye_left_x != float(-1)) {
                auto xy = project(bodyStruct.eye_left_y, bodyStruct.eye_left_z, -bodyStruct.eye_left_x);
                circle( img, cv::Point(std::get<0>(xy), std::get<1>(xy)), markerRadius*2, blue, cv::FILLED, cv::LINE_8 );
                std::cerr << "right knee: " << std::get<0>(xy) << " " << std::get<1>(xy) << std::endl << std::flush;
              }               
              if (bodyStruct.ear_left_x != float(-1)) {
                auto xy = project(bodyStruct.ear_left_y, bodyStruct.ear_left_z, -bodyStruct.ear_left_x);
                circle( img, cv::Point(std::get<0>(xy), std::get<1>(xy)), markerRadius*2, blue, cv::FILLED, cv::LINE_8 );
                std::cerr << "right knee: " << std::get<0>(xy) << " " << std::get<1>(xy) << std::endl << std::flush;
              }   
              if (bodyStruct.eye_right_x != float(-1)) {
                auto xy = project(bodyStruct.eye_right_y, bodyStruct.eye_right_z, -bodyStruct.eye_right_x);
                circle( img, cv::Point(std::get<0>(xy), std::get<1>(xy)), markerRadius*2, blue, cv::FILLED, cv::LINE_8 );
                std::cerr << "right knee: " << std::get<0>(xy) << " " << std::get<1>(xy) << std::endl << std::flush;
              }               
              if (bodyStruct.ear_right_x != float(-1)) {
                auto xy = project(bodyStruct.ear_right_y, bodyStruct.ear_right_z, -bodyStruct.ear_right_x);
                circle( img, cv::Point(std::get<0>(xy), std::get<1>(xy)), markerRadius*2, blue, cv::FILLED, cv::LINE_8 );
                std::cerr << "right knee: " << std::get<0>(xy) << " " << std::get<1>(xy) << std::endl << std::flush;
              } 
            }

            // take all detected 2D pixel coordinates and turn them red
            if(bodyStruct.neck_2d_conf > confidenceThreshold) {
              circle( img, cv::Point(bodyStruct.neck_2d_x, bodyStruct.neck_2d_y), markerRadius, cv::Scalar(0,0,int(double(bodyStruct.neck_2d_conf) * 255.0)), cv::FILLED, cv::LINE_8 );
              std::cerr << "neck 2d: " << bodyStruct.neck_2d_x << " " << bodyStruct.neck_2d_y << std::endl << std::flush;
            }
            if(bodyStruct.nose_2d_conf > confidenceThreshold) {
              circle( img, cv::Point(bodyStruct.nose_2d_x, bodyStruct.nose_2d_y), markerRadius, cv::Scalar(0,0,int(double(bodyStruct.nose_2d_conf) * 255.0)), cv::FILLED, cv::LINE_8 );
              showDistance(cv::Point(bodyStruct.nose_2d_x, bodyStruct.nose_2d_y), red, bodyStruct.nose_2d_depth);
              std::cerr << "nose 2d: " << bodyStruct.nose_2d_x << " " << bodyStruct.nose_2d_y << std::endl << std::flush;
            }
            // if(bodyStruct.pelvis_2d_conf > confidenceThreshold)
            //   circle( img, cv::Point(bodyStruct.pelvis_2d_x, bodyStruct.pelvis_2d_y), markerRadius, cv::Scalar(0,0,int(double(bodyStruct.pelvis_2d_conf) * 255.0)), cv::FILLED, cv::LINE_8 );
            if(bodyStruct.shoulder_left_2d_conf > confidenceThreshold)
              circle( img, cv::Point(bodyStruct.shoulder_left_2d_x, bodyStruct.shoulder_left_2d_y), markerRadius, cv::Scalar(0,0,int(double(bodyStruct.shoulder_left_2d_conf) * 255.0)), cv::FILLED, cv::LINE_8 );
            if(bodyStruct.elbow_left_2d_conf > confidenceThreshold)
              circle( img, cv::Point(bodyStruct.elbow_left_2d_x, bodyStruct.elbow_left_2d_y), markerRadius, cv::Scalar(0,0,int(double(bodyStruct.elbow_left_2d_conf) * 255.0)), cv::FILLED, cv::LINE_8 );
            if(bodyStruct.wrist_left_2d_conf > confidenceThreshold)
              circle( img, cv::Point(bodyStruct.wrist_left_2d_x, bodyStruct.wrist_left_2d_y), markerRadius, cv::Scalar(0,0,int(double(bodyStruct.wrist_left_2d_conf) * 255.0)), cv::FILLED, cv::LINE_8 );
            if(bodyStruct.hip_left_2d_conf > confidenceThreshold)
              circle( img, cv::Point(bodyStruct.hip_left_2d_x, bodyStruct.hip_left_2d_y), markerRadius, cv::Scalar(0,0,int(double(bodyStruct.hip_left_2d_conf) * 255.0)), cv::FILLED, cv::LINE_8 );
            if(bodyStruct.knee_left_2d_conf > confidenceThreshold)
              circle( img, cv::Point(bodyStruct.knee_left_2d_x, bodyStruct.knee_left_2d_y), markerRadius, cv::Scalar(0,0,int(double(bodyStruct.knee_left_2d_conf) * 255.0)), cv::FILLED, cv::LINE_8 );
            if(bodyStruct.ankle_left_2d_conf > confidenceThreshold)
              circle( img, cv::Point(bodyStruct.ankle_left_2d_x, bodyStruct.ankle_left_2d_y), markerRadius, cv::Scalar(0,0,int(double(bodyStruct.ankle_left_2d_conf) * 255.0)), cv::FILLED, cv::LINE_8 );
            if(bodyStruct.shoulder_right_2d_conf > confidenceThreshold)
              circle( img, cv::Point(bodyStruct.shoulder_right_2d_x, bodyStruct.shoulder_right_2d_y), markerRadius, cv::Scalar(0,0,int(double(bodyStruct.shoulder_right_2d_conf) * 255.0)), cv::FILLED, cv::LINE_8 );
            if(bodyStruct.elbow_right_2d_conf > confidenceThreshold)
              circle( img, cv::Point(bodyStruct.elbow_right_2d_x, bodyStruct.elbow_right_2d_y), markerRadius, cv::Scalar(0,0,int(double(bodyStruct.elbow_right_2d_conf) * 255.0)), cv::FILLED, cv::LINE_8 );
            if(bodyStruct.wrist_right_2d_conf > confidenceThreshold)
              circle( img, cv::Point(bodyStruct.wrist_right_2d_x, bodyStruct.wrist_right_2d_y), markerRadius, cv::Scalar(0,0,int(double(bodyStruct.wrist_right_2d_conf) * 255.0)), cv::FILLED, cv::LINE_8 );
            if(bodyStruct.hip_right_2d_conf > confidenceThreshold)
              circle( img, cv::Point(bodyStruct.hip_right_2d_x, bodyStruct.hip_right_2d_y), markerRadius, cv::Scalar(0,0,int(double(bodyStruct.hip_right_2d_conf) * 255.0)), cv::FILLED, cv::LINE_8 );
            if(bodyStruct.knee_right_2d_conf > confidenceThreshold)
              circle( img, cv::Point(bodyStruct.knee_right_2d_x, bodyStruct.knee_right_2d_y), markerRadius, cv::Scalar(0,0,int(double(bodyStruct.knee_right_2d_conf) * 255.0)), cv::FILLED, cv::LINE_8 );
            if(bodyStruct.ankle_right_2d_conf > confidenceThreshold)
              circle( img, cv::Point(bodyStruct.ankle_right_2d_x, bodyStruct.ankle_right_2d_y), markerRadius, cv::Scalar(0,0,int(double(bodyStruct.ankle_right_2d_conf) * 255.0)), cv::FILLED, cv::LINE_8 );
            if(bodyStruct.eye_left_2d_conf > confidenceThreshold)
              circle( img, cv::Point(bodyStruct.eye_left_2d_x, bodyStruct.eye_left_2d_y), markerRadius, cv::Scalar(0,0,int(double(bodyStruct.eye_left_2d_conf) * 255.0)), cv::FILLED, cv::LINE_8 );
            if(bodyStruct.ear_left_2d_conf > confidenceThreshold)
              circle( img, cv::Point(bodyStruct.ear_left_2d_x, bodyStruct.ear_left_2d_y), markerRadius, cv::Scalar(0,0,int(double(bodyStruct.ear_left_2d_conf) * 255.0)), cv::FILLED, cv::LINE_8 );
            if(bodyStruct.eye_right_2d_conf > confidenceThreshold)
              circle( img, cv::Point(bodyStruct.eye_right_x, bodyStruct.eye_right_y), markerRadius, cv::Scalar(0,0,int(double(bodyStruct.eye_right_2d_conf) * 255.0)), cv::FILLED, cv::LINE_8 );
            if(bodyStruct.ear_right_2d_conf > confidenceThreshold)
              circle( img, cv::Point(bodyStruct.ear_right_x, bodyStruct.ear_right_y), markerRadius, cv::Scalar(0,0,int(double(bodyStruct.ear_right_2d_conf) * 255.0)), cv::FILLED, cv::LINE_8 );
            
            //If neck is confident, make purple marker
            if(bodyStruct.neck_2d_conf > .5)
            {
              rectangle(img, cv::Point(bodyStruct.neck_2d_x-6, bodyStruct.neck_2d_y-6), cv::Point(bodyStruct.neck_2d_x+6, bodyStruct.neck_2d_y+6), cv::Scalar( 255, 0, 255 ), cv::FILLED, cv::LINE_8 );
            }
            //If neck not confident but shoulders are confident, choose half way point of shoulders
            else if (bodyStruct.shoulder_left_2d_conf > .5 && bodyStruct.shoulder_right_2d_conf > .5)
            {
              int x = (bodyStruct.shoulder_left_2d_x + bodyStruct.shoulder_right_2d_x)/2;
              int y = (bodyStruct.shoulder_left_2d_y + bodyStruct.shoulder_right_2d_y)/2;
              rectangle(img, cv::Point(x-6, y-6), cv::Point(x+6, y+6), cv::Scalar( 255, 0, 255 ), cv::FILLED, cv::LINE_8 );
            }
            //If that isn't true do midpoint between hips
            else if (bodyStruct.hip_left_2d_conf > .5 && bodyStruct.hip_right_2d_conf > .5)
            {
              int x = (bodyStruct.hip_left_2d_x + bodyStruct.hip_right_2d_x)/2;
              int y = (bodyStruct.hip_left_2d_y + bodyStruct.hip_right_2d_y)/2;
              rectangle(img, cv::Point(x-6, y-6), cv::Point(x+6, y+6), cv::Scalar( 255, 0, 255 ), cv::FILLED, cv::LINE_8 );
            }
            //If that isn't true last result do nose
            else if (bodyStruct.nose_2d_conf > 0)
            {
              rectangle(img, cv::Point(bodyStruct.nose_2d_x-6, bodyStruct.nose_2d_y-6), cv::Point(bodyStruct.nose_2d_x+6, bodyStruct.nose_2d_y+6), cv::Scalar( 255, 0, 255 ), cv::FILLED, cv::LINE_8 );
            }

            cv::imwrite("decorated-img-" + std::to_string(c) + ".jpg", img);
          }

#if HAS_IMSHOW
          cv::namedWindow(decoder_id);
          cv::imshow(decoder_id, img);
          cv::waitKey(1);
#endif
        }
      }

      c++;
#if 0
      if (> 100) {
        break;
      }
#endif      
    }

  } catch (std::exception &e) {
    spdlog::error(e.what());
  }

  return 0;
}

#ifndef SSP_PLUGIN
int main(int argc, char *argv[]) {

  spdlog::set_level(spdlog::level::debug);

  srand(time(NULL));
    
  if (argc < 2) {
    std::cerr << "Usage: ssp_client_opencv <port> (<log level>) (<log file>)"
              << std::endl;
    return 1;
  }
  std::string log_level = "debug";
  std::string log_file = "";

  if (argc > 2)
    log_level = argv[2];
  if (argc > 3)
    log_file = argv[3];

  int port = std::stoi(argv[1]);
    
  return ssp_client_opencv(port);
}
#endif


