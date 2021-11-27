/**
 * \file similarity_measures.h @brief Similarity measures
 */
// Created by amourao on 12-08-2019.
#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>

namespace moetsi::ssp {

using namespace cv;

/**
 * @brief Get Peak Signal to Noise Ration similarity
 * \param I1 image 1
 * \param I2 image 2
 * \param max_value max value in the PSNR formula
 * \return PSNR image similarity
 */
double GetPSNR(const Mat &I1, const Mat &I2, double max_value);

/**
 * @brief Get Mean Square Error (distance) between images
 * \param I1 image 1
 * \param I2 image 2
 * \return MSE between these 2 images
 */
double GetMSE(const Mat &I1, const Mat &I2);

/**
 * @brief Get Structural Similarity between 2 images
 * cf. for instance http://amroamroamro.github.io/mexopencv/opencv/image_similarity_demo.html for a simple SSIM introduction
 * \param I1 image 1
 * \param I2 image 2
 * \return 3 channel similarity measure
 */
Scalar GetMSSIM(const Mat &i1, const Mat &i2);

} // namespace moetsi::ssp
