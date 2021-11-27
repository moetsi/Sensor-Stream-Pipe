/**
 * \file zdepth_decoder.h @brief ZDepth decoder
 */
// Created by amourao on 23-09-2019.

#pragma once

#include "idecoder.h"
#include "zdepth.hpp"

namespace moetsi::ssp {

/**
 * @brief ZDepthDecoder ZDepth format decoder
 */
class ZDepthDecoder : public IDecoder {
private:
  std::vector<uint16_t> decompressed_buffer_;
  int width_;
  int height_;
  zdepth::DepthCompressor decompressor_;

public:
  /** @brief Constructor */
  ZDepthDecoder();
  /** @brief Destructor */
  ~ZDepthDecoder();
  /**
   *  @brief Initialize. 
   *  \param parameter_data parameters
   */ 
  void Init(std::vector<unsigned char> parameter_data);
  /**
   * @brief Extract an opencv image from a FrameStruct
   * \param data FrameStruct
   * \return OpenCV matrix/image
   */  
  cv::Mat Decode(FrameStruct& frame);
};

} // namespace moetsi::ssp
