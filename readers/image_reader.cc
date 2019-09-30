//
// Created by amourao on 26-06-2019.
//

#include "image_reader.h"

ImageReader::ImageReader(std::string filename) {
  // bundle_fusion_apt0;0;0;30

  // std::string sceneDesc;
  // unsigned int sensocpsrId;
  // unsigned int deviceId;
  try {

    std::ifstream file(filename);
    std::string line;
    getline(file, line);
    std::string value;

    std::stringstream ss(line);
    getline(ss, scene_desc_, ';');

    std::string sensor_id_str, device_id_str, frame_count_str, fps_str, frame_type_str;
    getline(ss, device_id_str, ';');
    getline(ss, sensor_id_str, ';');
    getline(ss, frame_type_str, ';');
    getline(ss, fps_str);

    sensor_id_ = std::stoul(sensor_id_str);
    device_id_ = std::stoul(device_id_str);
    frame_type_ = std::stoul(frame_type_str);
    fps_ = std::stoul(fps_str);

    // get frame count
    getline(file, frame_count_str);
    unsigned int frame_count = std::stoul(frame_count_str);

    while (getline(file, line))
      frame_lines_.push_back(line);

    if (frame_count != frame_lines_.size())
      spdlog::warn(
          "Lines read do not match expected size: {} read vs. {} expected.",
          frame_lines_.size(), frame_count);

    stream_id_ = RandomString(16);

    codec_params_struct_ = nullptr;
    current_frame_internal_ = nullptr;

  } catch (std::exception &e) {
    throw std::invalid_argument("Error reading frame file");
  }
  Reset();
}

ImageReader::~ImageReader() {
  if (codec_params_struct_ != nullptr)
    delete codec_params_struct_;
}

std::vector<unsigned char> ImageReader::ReadFile(std::string &filename) {
  std::streampos file_size;
  std::ifstream file(filename, std::ios::binary);

  file.seekg(0, std::ios::end);
  file_size = file.tellg();
  file.seekg(0, std::ios::beg);

  std::vector<unsigned char> file_data(file_size);
  file.read((char *)&file_data[0], file_size);
  return file_data;
}

FrameStruct *ImageReader::CreateFrameStruct(unsigned int frame_id) {
  // 0;/home/amourao/data/bundle_fusion/apt0/frame-000000.color.jpg;/home/amourao/data/bundle_fusion/apt0/frame-000000.color.jpg

  std::string line = frame_lines_[frame_id];
  std::stringstream ss(line);

  std::string frame_id_str, frame_path;

  getline(ss, frame_id_str, ';');
  getline(ss, frame_path);

  unsigned int read_frame_id = std::stoul(frame_id_str);

  if (read_frame_id != frame_counter_)
    spdlog::warn("Frame ids do not match: {} read vs. {} expected.",
                 read_frame_id, frame_counter_);

  std::vector<unsigned char> file_data = ReadFile(frame_path);
  FrameStruct *frame = new FrameStruct();

  frame->message_type = 0;

  frame->frame_data_type = 0;
  frame->scene_desc = scene_desc_;
  frame->device_id = device_id_;
  frame->sensor_id = sensor_id_;
  frame->frame_type = frame_type_;
  frame->timestamps.push_back(1000.0 / fps_ * frame_counter_);
  frame->timestamps.push_back(CurrentTimeMs());

  frame->frame_id = read_frame_id;

  frame->frame = file_data;
  frame->stream_id = stream_id_;

  if (codec_params_struct_ == nullptr) {
    ImageDecoder image_decoder;
    AVFrame *frame_av = av_frame_alloc();
    image_decoder.ImageBufferToAVFrame(frame, frame_av);
    codec_params_struct_ = new CodecParamsStruct(frame->codec_data);
    av_frame_free(&frame_av);
  }
  frame->codec_data = *codec_params_struct_;

  return frame;
}

unsigned int ImageReader::GetCurrentFrameId() { return frame_counter_; }

std::vector<FrameStruct *> ImageReader::GetCurrentFrame() {
  std::vector<FrameStruct *> v;
  v.push_back(current_frame_internal_);
  return v;
}

void ImageReader::NextFrame() {
  frame_counter_ += 1;
  current_frame_internal_ = CreateFrameStruct(frame_counter_);
}

bool ImageReader::HasNextFrame() {
  return frame_counter_ + 1 < frame_lines_.size();
}

void ImageReader::GoToFrame(unsigned int frame_id) {
  frame_counter_ = frame_id;
  current_frame_internal_ = CreateFrameStruct(frame_counter_);
}

void ImageReader::Reset() {
  frame_counter_ = 0;
  current_frame_internal_ = CreateFrameStruct(frame_counter_);
}

unsigned int ImageReader::GetFps() { return fps_; }

std::vector<unsigned int> ImageReader::GetType() {
  std::vector<unsigned int> result;
  result.push_back(frame_type_);
  return result;
}
