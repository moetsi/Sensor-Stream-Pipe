//
// Created by amourao on 26-06-2019.
//

#include "KinectReader.h"

std::atomic_bool exiting(false);

KinectReader::KinectReader(uint8_t _device_index,
                           ExtendedAzureConfig _device_config) {

  device_index = _device_index;

  device_config = _device_config.device_config;

  stream_color = _device_config.stream_color;
  stream_depth = _device_config.stream_depth;
  stream_ir = _device_config.stream_ir;

  absoluteExposureValue = _device_config.absoluteExposureValue;
  record_imu = false;

  streamId = randomString(16);

  const uint32_t installed_devices = k4a_device_get_installed_count();
  if (device_index >= installed_devices) {
    spdlog::error("Kinect Device not found.");
    exit(1);
  }

  if (K4A_FAILED(k4a_device_open(device_index, &device))) {
    spdlog::error("Runtime error: k4a_device_open() failed.");
    exit(1);
  }

  char serial_number_buffer[256];
  size_t serial_number_buffer_size = sizeof(serial_number_buffer);
  CHECK(k4a_device_get_serialnum(device, serial_number_buffer,
                                 &serial_number_buffer_size),
        device);

  k4a_hardware_version_t version_info;
  CHECK(k4a_device_get_version(device, &version_info), device);

  spdlog::info("Kinect Device serial number: {}", serial_number_buffer);
  spdlog::info("Kinect Device version: {}; C: {}.{}.{}; D: {}.{}.{} {}.{}; A: "
               "{}.{}.{};  ",
               (version_info.firmware_build == K4A_FIRMWARE_BUILD_RELEASE
                    ? "Rel"
                    : "Dbg"),
               version_info.rgb.major, version_info.rgb.minor,
               version_info.rgb.iteration, version_info.depth.major,
               version_info.depth.minor, version_info.depth.iteration,
               version_info.depth_sensor.major, version_info.depth_sensor.minor,
               version_info.audio.major, version_info.audio.minor,
               version_info.audio.iteration);

  uint32_t camera_fps = k4a_convert_fps_to_uint(device_config.camera_fps);

  if (camera_fps <= 0 ||
      (device_config.color_resolution == K4A_COLOR_RESOLUTION_OFF &&
       device_config.depth_mode == K4A_DEPTH_MODE_OFF)) {
    spdlog::error("Either the color or depth modes must be enabled to record.");
    exit(1);
  }

  if (absoluteExposureValue != 0) {
    if (K4A_FAILED(k4a_device_set_color_control(
            device, K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE,
            K4A_COLOR_CONTROL_MODE_MANUAL, absoluteExposureValue))) {
      spdlog::error("Runtime error: k4a_device_set_color_control() failed.");
      exit(1);
    }
  } else {
    if (K4A_FAILED(k4a_device_set_color_control(
            device, K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE,
            K4A_COLOR_CONTROL_MODE_AUTO, 0))) {
      spdlog::error("Runtime error: k4a_device_set_color_control() failed.");
      exit(1);
    }
  }

  CHECK(k4a_device_start_cameras(device, &device_config), device);
  if (record_imu) {
    CHECK(k4a_device_start_imu(device), device);
  }

  spdlog::info("Kinect Device started");

  // Wait for the first capture before starting recording.
  int32_t timeout_sec_for_first_capture = 60;
  if (device_config.wired_sync_mode == K4A_WIRED_SYNC_MODE_SUBORDINATE) {
    timeout_sec_for_first_capture = 360;
    spdlog::warn("[subordinate mode] Waiting for signal from master");
  }
  clock_t first_capture_start = clock();
  // Wait for the first capture in a loop so Ctrl-C will still exit.
  while (!exiting && (clock() - first_capture_start) <
                         (CLOCKS_PER_SEC * timeout_sec_for_first_capture)) {
    result = k4a_device_get_capture(device, &capture, 100);
    if (result == K4A_WAIT_RESULT_SUCCEEDED) {
      k4a_capture_release(capture);
      break;
    } else if (result == K4A_WAIT_RESULT_FAILED) {
      spdlog::error(
          "Runtime error: k4a_device_get_capture() returned error: {}", result);
      exit(1);
    }
  }

  if (exiting) {
    spdlog::error("Exiting");
    k4a_device_close(device);
    exit(0);
  } else if (result == K4A_WAIT_RESULT_TIMEOUT) {
    spdlog::error("Timed out waiting for first capture.");
    exit(1);
  }

  ccs = new CameraCalibrationStruct();

  size_t buffer_size = 10000;
  ccs->type = 0;
  ccs->data.resize(buffer_size);
  k4a_buffer_result_t output_buffer_size = k4a_device_get_raw_calibration(
      device, (uint8_t *)ccs->data.data(), &buffer_size);
  ccs->data.resize(buffer_size + 1);
  ccs->data[buffer_size] = '\0';

  ccs->extra_data.push_back(_device_config.device_config.depth_mode);
  ccs->extra_data.push_back(_device_config.device_config.color_format);

  recording_start = clock();
  timeout_ms = 1000 / camera_fps;

  frameTemplate.frameId = 0;
  frameTemplate.deviceId = 0;

  frameTemplate.messageType = 0;

  frameTemplate.frameDataType = 0;
  frameTemplate.sceneDesc = "kinect";
  frameTemplate.streamId = randomString(16);

  currentFrameCounter.push_back(0);
  currentFrameCounter.push_back(0);
  currentFrameCounter.push_back(0);

  cpss.push_back(nullptr);
  cpss.push_back(nullptr);
  cpss.push_back(nullptr);
}

KinectReader::~KinectReader() {
  k4a_device_stop_cameras(device);

  k4a_device_close(device);

  for (uint i = 0; i < cpss.size(); i++)
    if (cpss[i] != nullptr)
      delete cpss[i];

  for (uint i = 0; i < currFrame.size(); i++)
    if (cpss[i] != nullptr)
      delete currFrame[i];
}

void KinectReader::nextFrame() {

  currFrame.clear();

  do {
    result = k4a_device_get_capture(device, &capture, timeout_ms);
    if (result == K4A_WAIT_RESULT_TIMEOUT) {
      continue;
    } else if (result != K4A_WAIT_RESULT_SUCCEEDED) {
      spdlog::error(
          "Runtime error: k4a_device_get_capture() returned error: {}", result);
      break;
    }
    uint64_t capture_timestamp = currentTimeMs();
    if (stream_color &&
        device_config.color_resolution != K4A_COLOR_RESOLUTION_OFF) {
      k4a_image_t colorImage = k4a_capture_get_color_image(capture);
      if (colorImage && k4a_image_get_format(colorImage) != 6) {
        FrameStruct *s = new FrameStruct(frameTemplate);
        s->sensorId = 0;
        s->frameType = 0;
        s->frameId = currentFrameCounter.at(0)++;
        s->timestamps.push_back(
            k4a_image_get_device_timestamp_usec(colorImage) / 1000);
        s->timestamps.push_back(capture_timestamp);

        uint8_t *buffer = k4a_image_get_buffer(colorImage);
        size_t size = k4a_image_get_size(colorImage);

        if (k4a_image_get_format(colorImage) == K4A_IMAGE_FORMAT_COLOR_MJPG) {
          s->frameDataType = 0;
          s->frame = std::vector<uchar>(buffer, buffer + size);
          if (cpss.at(0) == nullptr) {
            ImageDecoder id;
            AVFrame *avframe = av_frame_alloc();
            id.imageBufferToAVFrame(s, avframe);
            cpss.at(0) = new CodecParamsStruct(s->codec_data);
            av_frame_free(&avframe);
          }
          s->camera_calibration_data = *ccs;
          s->codec_data = *cpss.at(0);
        } else {
          s->frameDataType = 2;

          int rows = k4a_image_get_height_pixels(colorImage);
          int cols = k4a_image_get_width_pixels(colorImage);

          s->frame.resize(size + 2 * sizeof(int));

          memcpy(&s->frame[0], &cols, sizeof(int));
          memcpy(&s->frame[4], &rows, sizeof(int));
          memcpy(&s->frame[8], buffer, size);
        }
        currFrame.push_back(s);
        if (colorImage != nullptr)
          k4a_image_release(colorImage);
      }
    }

    if (stream_depth && device_config.depth_mode != K4A_DEPTH_MODE_OFF) {

      k4a_image_t depthImage = k4a_capture_get_depth_image(capture);
      if (depthImage && k4a_image_get_format(depthImage) != 6) {

        FrameStruct *s = new FrameStruct(frameTemplate);
        s->sensorId = 1;
        s->frameType = 1;
        s->frameDataType = 3;
        s->frameId = currentFrameCounter.at(1)++;
        s->timestamps.push_back(
            k4a_image_get_device_timestamp_usec(depthImage) / 1000);
        s->timestamps.push_back(capture_timestamp);
        s->camera_calibration_data = *ccs;
        uint8_t *buffer = k4a_image_get_buffer(depthImage);
        size_t size = k4a_image_get_size(depthImage);
        // convert the raw buffer to cv::Mat
        int rows = k4a_image_get_height_pixels(depthImage);
        int cols = k4a_image_get_width_pixels(depthImage);

        s->frame.resize(size + 2 * sizeof(int));

        memcpy(&s->frame[0], &cols, sizeof(int));
        memcpy(&s->frame[4], &rows, sizeof(int));
        memcpy(&s->frame[8], buffer, size);

        currFrame.push_back(s);
        if (depthImage != nullptr)
          k4a_image_release(depthImage);
      }
    }

    if (stream_ir && device_config.depth_mode != K4A_DEPTH_MODE_OFF) {

      k4a_image_t irImage = k4a_capture_get_ir_image(capture);
      if (irImage && k4a_image_get_format(irImage) != 6) {
        FrameStruct *s = new FrameStruct(frameTemplate);
        s->sensorId = 2;
        s->frameType = 2;
        s->frameId = currentFrameCounter.at(2)++;
        s->frameDataType = 3;
        s->timestamps.push_back(k4a_image_get_device_timestamp_usec(irImage) /
                                1000);
        s->timestamps.push_back(capture_timestamp);
        s->camera_calibration_data = *ccs;
        uint8_t *buffer = k4a_image_get_buffer(irImage);
        size_t size = k4a_image_get_size(irImage);
        // convert the raw buffer to cv::Mat
        int rows = k4a_image_get_height_pixels(irImage);
        int cols = k4a_image_get_width_pixels(irImage);

        s->frame.resize(size + 2 * sizeof(int));

        memcpy(&s->frame[0], &cols, sizeof(int));
        memcpy(&s->frame[4], &rows, sizeof(int));
        memcpy(&s->frame[8], buffer, size);

        currFrame.push_back(s);
        if (irImage != nullptr)
          k4a_image_release(irImage);
      }
    }
    k4a_capture_release(capture);
  } while (result == K4A_WAIT_RESULT_FAILED);
}

bool KinectReader::hasNextFrame() { return true; }

void KinectReader::reset() {}

std::vector<FrameStruct *> KinectReader::currentFrame() { return currFrame; }

FrameStruct *KinectReader::currentFrame(uint type) {
  for (FrameStruct *fs : currFrame) {
    if (fs->frameType == type)
      return fs;
  }
  return NULL;
}

uint KinectReader::getFps() {
  if (device_config.camera_fps == K4A_FRAMES_PER_SECOND_5)
    return 5;
  if (device_config.camera_fps == K4A_FRAMES_PER_SECOND_15)
    return 15;
  if (device_config.camera_fps == K4A_FRAMES_PER_SECOND_30)
    return 30;
  return -1;
}

std::vector<uint> KinectReader::getType() {
  std::vector<uint> types;

  if (stream_color) {
    types.push_back(0);
  }
  if (stream_depth) {
    types.push_back(1);
  }
  if (stream_ir) {
    types.push_back(2);
  }

  return types;
}

void KinectReader::goToFrame(uint frameId) {}
uint KinectReader::currentFrameId() { return currentFrameCounter.at(0); }
