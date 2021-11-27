/**
 * \file idecoder.h @brief Frame decoder interface
 */

// extra annotations
/**
 * \file decoders/idecoder.h @brief Frame decoder interface
 */
/**
 * \file include/decoders/idecoder.h @brief Frame decoder interface
 */

// Created by amourao on 12-09-2019.
#pragma once

#include "../structs/frame_struct.h"
#include <opencv2/core/mat.hpp>

namespace moetsi::ssp {

/**
 * @brief IDecoder abstract decoder interface
 */
class IDecoder {
public:
  /** @brief Virtual destructor */
  virtual ~IDecoder() {}
  /**
   * @brief Extract an opencv image from a FrameStruct
   * \param data FrameStruct
   * \return OpenCV matrix/image
   */
  virtual cv::Mat Decode(FrameStruct& data) = 0;
};

/**
 * @brief IDecoder factory.
 * \param config configuration
 * \return IDecoder instance
 */
std::shared_ptr<IDecoder> IDecoderFactory(const std::string & config);

} // moetsi::ssp
