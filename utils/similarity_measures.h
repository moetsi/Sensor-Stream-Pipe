//
// Created by amourao on 12-08-2019.
//

#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;

double GetPSNR(const Mat &I1, const Mat &I2, double max_value);

double GetMSE(const Mat &I1, const Mat &I2);

Scalar GetMSSIM(const Mat &i1, const Mat &i2);
