//
// Created by amourao on 28-10-2019.
//

#pragma once

#include "../decoders/idecoder.h"
#include "../decoders/libav_decoder.h"
#include "../structs/frame_struct.hpp"
#include "../structs/body_struct.hpp"
#include <opencv2/core/mat.hpp>

bool FrameStructToMat(FrameStruct &f, cv::Mat &img,
                      std::unordered_map<std::string, std::shared_ptr<IDecoder>> &decoders);