/**
 * \file nv_decoder.h @brief NvPipe decoder
 */
// Created by amourao on 12-09-2019
#pragma once

#include <NvPipe.h>
#include <iostream>
#include <opencv2/core/mat.hpp>

#include "../utils/nvpipe_types.h"
#include "../utils/video_utils.h"
#include "idecoder.h"

namespace moetsi::ssp {

/**
 * @brief NvPipe decoder
 */
class NvDecoder : public IDecoder {
private:
  NvPipe* decoder_;
  std::vector<uint8_t> decompressed_buffer_;
  unsigned int width_;
  unsigned int height_;
  NvPipe_Codec codec_;
  NvPipe_Format format_;

public:
  /** @brief Constructor */
  NvDecoder();
  /** @brief Destructor */
  ~NvDecoder();
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
