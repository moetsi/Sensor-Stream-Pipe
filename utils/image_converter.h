/**
 * \file image_converter.h @brief Image converter from frame struct to opencv 
 */
// Created by amourao on 28-10-2019.
#pragma once

#include "../decoders/idecoder.h"
#include "../structs/frame_struct.h"

#include <opencv2/core/mat.hpp>

namespace moetsi::ssp {

/**
 * @brief Convert frame struct to opencv matrix.
 * \param f Frame struct
 * \param img Target opencv image
 * \param decoders decoder dictionary
 */
bool FrameStructToMat(FrameStruct &f, cv::Mat &img,
                      std::unordered_map<std::string, std::shared_ptr<IDecoder>> &decoders);

} // namespace moetsi::ssp
