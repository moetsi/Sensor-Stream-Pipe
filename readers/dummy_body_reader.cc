//
// Created by amourao on 26-06-2019.
//

#include "dummy_body_reader.h"


DummyBodyReader::DummyBodyReader() {

  stream_id_ = RandomString(16);


  frame_template_.frame_id = 0;
  frame_template_.device_id = 0;

  frame_template_.message_type = 0;

  frame_template_.frame_data_type = 0;
  frame_template_.scene_desc = "dummybody";
  frame_template_.stream_id = RandomString(16);

  frame_counter_.push_back(0);
  frame_counter_.push_back(0);
  frame_counter_.push_back(0);

  codec_params_structs_.push_back(nullptr);
  codec_params_structs_.push_back(nullptr);
  codec_params_structs_.push_back(nullptr);
}

DummyBodyReader::~DummyBodyReader() {
}

void DummyBodyReader::NextFrame() {

  current_frame_.clear();


  uint64_t capture_timestamp = CurrentTimeMs();
  if (stream_color_ &&
      device_config_.color_resolution != K4A_COLOR_RESOLUTION_OFF) {
    k4a_image_t color_image = k4a_capture_get_color_image(capture_);
    if (color_image && k4a_image_get_format(color_image) != 6) {
      std::shared_ptr<FrameStruct> s =
          std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
      s->sensor_id = 0;
      s->frame_type = 0;
      s->frame_id = frame_counter_.at(0)++;
      s->timestamps.push_back(
          k4a_image_get_device_timestamp_usec(color_image) / 1000);
      s->timestamps.push_back(capture_timestamp);

      uint8_t *buffer = k4a_image_get_buffer(color_image);
      size_t size = k4a_image_get_size(color_image);

      if (k4a_image_get_format(color_image) == K4A_IMAGE_FORMAT_COLOR_MJPG) {
        s->frame_data_type = 0;
        s->frame = std::vector<uchar>(buffer, buffer + size);
        if (codec_params_structs_.at(0) == nullptr) {
          ImageDecoder id;
          AVFrame *avframe_tmp = av_frame_alloc();
          AVFrameSharedP avframe =
              std::shared_ptr<AVFrame>(avframe_tmp, AVFrameSharedDeleter);
          id.ImageBufferToAVFrame(s, avframe);
          codec_params_structs_.at(0) = std::shared_ptr<CodecParamsStruct>(
              new CodecParamsStruct(s->codec_data));
        }
        s->camera_calibration_data = *camera_calibration_struct_;

        s->codec_data = *codec_params_structs_.at(0);
      } else {
        s->frame_data_type = 2;

        int rows = k4a_image_get_height_pixels(color_image);
        int cols = k4a_image_get_width_pixels(color_image);

        s->frame.resize(size + 2 * sizeof(int));

        memcpy(&s->frame[0], &cols, sizeof(int));
        memcpy(&s->frame[4], &rows, sizeof(int));
        memcpy(&s->frame[8], buffer, size);
      }
      current_frame_.push_back(s);
      if (color_image != nullptr)
        k4a_image_release(color_image);
    }
  }

}

bool DummyBodyReader::HasNextFrame() { return true; }

void DummyBodyReader::Reset() {}

std::vector<std::shared_ptr<FrameStruct>> DummyBodyReader::GetCurrentFrame() {
  return current_frame_;
}

unsigned int DummyBodyReader::GetFps() {
  return 10;
}

std::vector<unsigned int> DummyBodyReader::GetType() {
  std::vector<unsigned int> types;

  types.push_back(4);
  return types;
}

void DummyBodyReader::GoToFrame(unsigned int frame_id) {}
unsigned int DummyBodyReader::GetCurrentFrameId() { return frame_counter_.at(0); }
