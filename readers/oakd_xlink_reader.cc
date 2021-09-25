//
// Created by amourao on 26-06-2019.
//

#include "oakd_xlink_reader.h"

// // NONSENSE!
//   // Create pipeline
//   // Closer-in minimum depth, disparity range is doubled (from 95 to 190):
//   static std::atomic<bool> extended_disparity{false};
//   // Better accuracy for longer distance, fractional disparity 32-levels:
//   static std::atomic<bool> subpixel{false};
//   // Better handling for occlusions:
//   static std::atomic<bool> lr_check{false};
//   dai::Pipeline pipeline;

//   // Define sources and outputs
//   auto monoLeft = pipeline.create<dai::node::MonoCamera>();
//   auto monoRight = pipeline.create<dai::node::MonoCamera>();
//   auto depth = pipeline.create<dai::node::StereoDepth>();
//   auto xout = pipeline.create<dai::node::XLinkOut>();

//   xout->setStreamName("disparity");

//   // Properties
//   monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
//   monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
//   monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
//   monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

//   // Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
//   depth->initialConfig.setConfidenceThreshold(200);
//   // Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
//   depth->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
//   depth->setLeftRightCheck(lr_check);
//   depth->setExtendedDisparity(extended_disparity);
//   depth->setSubpixel(subpixel);

//   // Linking
//   monoLeft->out.link(depth->left);
//   monoRight->out.link(depth->right);
//   depth->disparity.link(xout->input);

//   // Connect to device and start pipeline
//   dai::Device device(pipeline);

//   // Output queue will be used to get the disparity frames from the outputs defined above
//   auto q = device.getOutputQueue("disparity", 4, false);

//   while(true) {
//       auto inDepth = q->get<dai::ImgFrame>();
//       auto frame = inDepth->getFrame();
//       // Normalization for better visualization
//       frame.convertTo(frame, CV_8UC1, 255 / depth->getMaxDisparity());

//       cv::imshow("disparity", frame);

//       // Available color maps: https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html
//       cv::applyColorMap(frame, frame, cv::COLORMAP_JET);
//       cv::imshow("disparity_color", frame);

//       int key = cv::waitKey(1);
//       if(key == 'q' || key == 'Q') {
//           return 0;
//       }
//   }
//   return 0;
//   // END OF NONSENSE!

// std::atomic_bool exiting(false);

OakdXlinkReader::OakdXlinkReader() {

  // device_index_ = _device_index;

  // device_config_ = _device_config.device_config;

  // stream_color_ = _device_config.stream_color;
  // stream_depth_ = _device_config.stream_depth;
  // stream_ir_ = _device_config.stream_ir;

  // absolute_exposure_value_ = _device_config.absolute_exposure_value;
  // record_imu_ = false;

  // stream_id_ = RandomString(16);

  // const uint32_t installed_devices = k4a_device_get_installed_count();
  // if (device_index_ >= installed_devices) {
  //   spdlog::error("Kinect Device not found.");
  //   exit(1);
  // }

  // if (K4A_FAILED(k4a_device_open(device_index_, &device_))) {
  //   spdlog::error("Runtime error: k4a_device_open() failed.");
  //   exit(1);
  // }

  // char serial_number_buffer[256];
  // size_t serial_number_buffer_size = sizeof(serial_number_buffer);
  // CHECK(k4a_device_get_serialnum(device_, serial_number_buffer,
  //                                &serial_number_buffer_size),
  //       device_);

  // k4a_hardware_version_t version_info;
  // CHECK(k4a_device_get_version(device_, &version_info), device_);

  // spdlog::info("Kinect Device serial number: {}", serial_number_buffer);
  // spdlog::info("Kinect Device version: {}; C: {}.{}.{}; D: {}.{}.{} {}.{}; A: "
  //              "{}.{}.{};  ",
  //              (version_info.firmware_build == K4A_FIRMWARE_BUILD_RELEASE
  //                   ? "Rel"
  //                   : "Dbg"),
  //              version_info.rgb.major, version_info.rgb.minor,
  //              version_info.rgb.iteration, version_info.depth.major,
  //              version_info.depth.minor, version_info.depth.iteration,
  //              version_info.depth_sensor.major, version_info.depth_sensor.minor,
  //              version_info.audio.major, version_info.audio.minor,
  //              version_info.audio.iteration);

  // uint32_t camera_fps = k4a_convert_fps_to_uint(device_config_.camera_fps);

  // if (camera_fps <= 0 ||
  //     (device_config_.color_resolution == K4A_COLOR_RESOLUTION_OFF &&
  //      device_config_.depth_mode == K4A_DEPTH_MODE_OFF)) {
  //   spdlog::error("Either the color or depth modes must be enabled to record.");
  //   exit(1);
  // }

  // if (absolute_exposure_value_ != 0) {
  //   if (K4A_FAILED(k4a_device_set_color_control(
  //           device_, K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE,
  //           K4A_COLOR_CONTROL_MODE_MANUAL, absolute_exposure_value_))) {
  //     spdlog::error("Runtime error: k4a_device_set_color_control() failed.");
  //     exit(1);
  //   }
  // } else {
  //   if (K4A_FAILED(k4a_device_set_color_control(
  //           device_, K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE,
  //           K4A_COLOR_CONTROL_MODE_AUTO, 0))) {
  //     spdlog::error("Runtime error: k4a_device_set_color_control() failed.");
  //     exit(1);
  //   }
  // }

  // CHECK(k4a_device_start_cameras(device_, &device_config_), device_);
  // if (record_imu_) {
  //   CHECK(k4a_device_start_imu(device_), device_);
  // }

  // spdlog::info("Kinect Device started");

  // // Wait for the first capture before starting recording.
  // int32_t timeout_sec_for_first_capture = 60;
  // if (device_config_.wired_sync_mode == K4A_WIRED_SYNC_MODE_SUBORDINATE) {
  //   timeout_sec_for_first_capture = 360;
  //   spdlog::warn("[subordinate mode] Waiting for signal from master");
  // }
  // clock_t first_capture_start = clock();
  // // Wait for the first capture in a loop so Ctrl-C will still exit.
  // while (!exiting && (clock() - first_capture_start) <
  //                        (CLOCKS_PER_SEC * timeout_sec_for_first_capture)) {
  //   result_ = k4a_device_get_capture(device_, &capture_, 100);
  //   if (result_ == K4A_WAIT_RESULT_SUCCEEDED) {
  //     k4a_capture_release(capture_);
  //     break;
  //   } else if (result_ == K4A_WAIT_RESULT_FAILED) {
  //     spdlog::error(
  //         "Runtime error: k4a_device_get_capture() returned error: {}",
  //         result_);
  //     exit(1);
  //   }
  // }

  // if (exiting) {
  //   spdlog::error("Exiting");
  //   k4a_device_close(device_);
  //   exit(0);
  // } else if (result_ == K4A_WAIT_RESULT_TIMEOUT) {
  //   spdlog::error("Timed out waiting for first capture.");
  //   exit(1);
  // }

  // camera_calibration_struct_ =
  //     std::shared_ptr<CameraCalibrationStruct>(new CameraCalibrationStruct());

  // size_t buffer_size = 10000;
  // camera_calibration_struct_->type = 0;
  // camera_calibration_struct_->data.resize(buffer_size);
  // k4a_buffer_result_t output_buffer_size = k4a_device_get_raw_calibration(
  //     device_, (uint8_t *)camera_calibration_struct_->data.data(),
  //     &buffer_size);

  // camera_calibration_struct_->data.resize(buffer_size);
  // // camera_calibration_struct_->data.resize(buffer_size + 1);
  // // camera_calibration_struct_->data[buffer_size] = '\0';
  // camera_calibration_struct_->extra_data.push_back(
  //     _device_config.device_config.depth_mode);
  // camera_calibration_struct_->extra_data.push_back(
  //     _device_config.device_config.color_resolution);

  // timeout_ms_ = 1000 / camera_fps;

  // frame_template_.frame_id = 0;
  // frame_template_.device_id = 0;

  // frame_template_.message_type = 0;

  // frame_template_.frame_data_type = 0;
  // frame_template_.scene_desc = "kinect";
  // frame_template_.stream_id = RandomString(16);

  // frame_counter_.push_back(0);
  // frame_counter_.push_back(0);
  // frame_counter_.push_back(0);

  // codec_params_structs_.push_back(nullptr);
  // codec_params_structs_.push_back(nullptr);
  // codec_params_structs_.push_back(nullptr);
}

OakdXlinkReader::~OakdXlinkReader() {
  // k4a_device_stop_cameras(device_);

  // k4a_device_close(device_);
}

void OakdXlinkReader::NextFrame() {

  // current_frame_.clear();

  // do {
  //   result_ = k4a_device_get_capture(device_, &capture_, timeout_ms_);
  //   if (result_ == K4A_WAIT_RESULT_TIMEOUT) {
  //     continue;
  //   } else if (result_ != K4A_WAIT_RESULT_SUCCEEDED) {
  //     spdlog::error(
  //         "Runtime error: k4a_device_get_capture() returned error: {}",
  //         result_);
  //     break;
  //   }
  //   uint64_t capture_timestamp = CurrentTimeMs();
  //   if (stream_color_ &&
  //       device_config_.color_resolution != K4A_COLOR_RESOLUTION_OFF) {
  //     k4a_image_t color_image = k4a_capture_get_color_image(capture_);
  //     if (color_image && k4a_image_get_format(color_image) != 6) {
  //       std::shared_ptr<FrameStruct> s =
  //           std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
  //       s->sensor_id = 0;
  //       s->frame_type = 0;
  //       s->frame_id = frame_counter_.at(0)++;
  //       s->timestamps.push_back(
  //           k4a_image_get_device_timestamp_usec(color_image) / 1000);
  //       s->timestamps.push_back(capture_timestamp);

  //       uint8_t *buffer = k4a_image_get_buffer(color_image);
  //       size_t size = k4a_image_get_size(color_image);

  //       if (k4a_image_get_format(color_image) == K4A_IMAGE_FORMAT_COLOR_MJPG) {
  //         s->frame_data_type = 0;
  //         s->frame = std::vector<uchar>(buffer, buffer + size);
  //         if (codec_params_structs_.at(0) == nullptr) {
  //           ImageDecoder id;
  //           AVFrame *avframe_tmp = av_frame_alloc();
  //           AVFrameSharedP avframe =
  //               std::shared_ptr<AVFrame>(avframe_tmp, AVFrameSharedDeleter);
  //           id.ImageBufferToAVFrame(s, avframe);
  //           codec_params_structs_.at(0) = std::shared_ptr<CodecParamsStruct>(
  //               new CodecParamsStruct(s->codec_data));
  //         }
  //         s->camera_calibration_data = *camera_calibration_struct_;

  //         s->codec_data = *codec_params_structs_.at(0);
  //       } else {
  //         s->frame_data_type = 2;

  //         int rows = k4a_image_get_height_pixels(color_image);
  //         int cols = k4a_image_get_width_pixels(color_image);

  //         s->frame.resize(size + 2 * sizeof(int));

  //         memcpy(&s->frame[0], &cols, sizeof(int));
  //         memcpy(&s->frame[4], &rows, sizeof(int));
  //         memcpy(&s->frame[8], buffer, size);
  //       }
  //       current_frame_.push_back(s);
  //       if (color_image != nullptr)
  //         k4a_image_release(color_image);
  //     }
  //   }

  //   if (stream_depth_ && device_config_.depth_mode != K4A_DEPTH_MODE_OFF) {

  //     k4a_image_t depth_image = k4a_capture_get_depth_image(capture_);
  //     if (depth_image && k4a_image_get_format(depth_image) != 6) {

  //       std::shared_ptr<FrameStruct> s =
  //           std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
  //       s->sensor_id = 1;
  //       s->frame_type = 1;
  //       s->frame_data_type = 3;
  //       s->frame_id = frame_counter_.at(1)++;
  //       s->timestamps.push_back(
  //           k4a_image_get_device_timestamp_usec(depth_image) / 1000);
  //       s->timestamps.push_back(capture_timestamp);
  //       s->camera_calibration_data = *camera_calibration_struct_;
  //       uint8_t *buffer = k4a_image_get_buffer(depth_image);
  //       size_t size = k4a_image_get_size(depth_image);
  //       // convert the raw buffer to cv::Mat
  //       int rows = k4a_image_get_height_pixels(depth_image);
  //       int cols = k4a_image_get_width_pixels(depth_image);

  //       s->frame.resize(size + 2 * sizeof(int));

  //       memcpy(&s->frame[0], &cols, sizeof(int));
  //       memcpy(&s->frame[4], &rows, sizeof(int));
  //       memcpy(&s->frame[8], buffer, size);

  //       current_frame_.push_back(s);
  //       if (depth_image != nullptr)
  //         k4a_image_release(depth_image);
  //     }
  //   }

  //   if (stream_ir_ && device_config_.depth_mode != K4A_DEPTH_MODE_OFF) {

  //     k4a_image_t ir_image = k4a_capture_get_ir_image(capture_);
  //     if (ir_image && k4a_image_get_format(ir_image) != 6) {
  //       std::shared_ptr<FrameStruct> s =
  //           std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
  //       s->sensor_id = 2;
  //       s->frame_type = 2;
  //       s->frame_id = frame_counter_.at(2)++;
  //       s->frame_data_type = 3;
  //       s->timestamps.push_back(k4a_image_get_device_timestamp_usec(ir_image) /
  //                               1000);
  //       s->timestamps.push_back(capture_timestamp);
  //       s->camera_calibration_data = *camera_calibration_struct_;
  //       uint8_t *buffer = k4a_image_get_buffer(ir_image);
  //       size_t size = k4a_image_get_size(ir_image);
  //       // convert the raw buffer to cv::Mat
  //       int rows = k4a_image_get_height_pixels(ir_image);
  //       int cols = k4a_image_get_width_pixels(ir_image);

  //       s->frame.resize(size + 2 * sizeof(int));

  //       memcpy(&s->frame[0], &cols, sizeof(int));
  //       memcpy(&s->frame[4], &rows, sizeof(int));
  //       memcpy(&s->frame[8], buffer, size);

  //       current_frame_.push_back(s);
  //       if (ir_image != nullptr)
  //         k4a_image_release(ir_image);
  //     }
  //   }
  //   k4a_capture_release(capture_);
  // } while (result_ == K4A_WAIT_RESULT_FAILED);
}

bool OakdXlinkReader::HasNextFrame() { return true; }

void OakdXlinkReader::Reset() {}

std::vector<std::shared_ptr<FrameStruct>> OakdXlinkReader::GetCurrentFrame() {
  // return current_frame_;
}

unsigned int OakdXlinkReader::GetFps() {
  // if (device_config_.camera_fps == K4A_FRAMES_PER_SECOND_5)
  //   return 5;
  // if (device_config_.camera_fps == K4A_FRAMES_PER_SECOND_15)
  //   return 15;
  // if (device_config_.camera_fps == K4A_FRAMES_PER_SECOND_30)
  //   return 30;
  // return -1;
}

std::vector<unsigned int> OakdXlinkReader::GetType() {
  std::vector<unsigned int> types;

  // if (stream_color_) {
  //   types.push_back(0);
  // }
  // if (stream_depth_) {
  //   types.push_back(1);
  // }
  // if (stream_ir_) {
  //   types.push_back(2);
  // }

  return types;
}

void OakdXlinkReader::GoToFrame(unsigned int frame_id) {}
unsigned int OakdXlinkReader::GetCurrentFrameId() {}
