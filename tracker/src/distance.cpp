// Copyright (C) 2018-2023 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
//

#include "../include/distance.hpp"

#include <stddef.h>

#include <cmath>
#include <vector>
#include <iostream>

#include "../include/logging.hpp"

// Constructor for CosDistance class, initializes the descriptor size and checks if it's valid.
CosDistance::CosDistance(const cv::Size& descriptor_size) : descriptor_size_(descriptor_size) {
    PT_CHECK(descriptor_size.area() != 0);
}

// Compute the cosine distance between two descriptors.
// A high return value (close to 1) indicates that the descriptors are very different.
// A low return value (close to 0) indicates that the descriptors are very similar.
float CosDistance::Compute(const cv::Mat& descr1, const cv::Mat& descr2) {
    // Ensure the descriptors are not empty and match the expected size.
    PT_CHECK(!descr1.empty());
    PT_CHECK(!descr2.empty());
    // Print the size of descr1 and descr2
    std::cout << "descr1.size(): " << descr1.size() << ", descriptor_size_: " << descriptor_size_ << std::endl;
    std::cout << "descr2.size(): " << descr2.size() << ", descriptor_size_: " << descriptor_size_ << std::endl;
    std::cout << "descriptor_size_: " << descriptor_size_ << std::endl;
    PT_CHECK(descr1.size() == descriptor_size_);
    PT_CHECK(descr2.size() == descriptor_size_);

    // Calculate the dot products and norms.
    double xy = descr1.dot(descr2);
    double xx = descr1.dot(descr1);
    double yy = descr2.dot(descr2);
    double norm = sqrt(xx * yy) + 1e-6; // Add a small value to avoid division by zero.

    // Return the cosine distance.
    // A high value (close to 1) means the vectors are dissimilar.
    // A low value (close to 0) means the vectors are similar.
    return 0.5f * static_cast<float>(1.0 - xy / norm);
}

// Compute the cosine distances between pairs of descriptors from two vectors.
// Returns a vector of distances.
std::vector<float> CosDistance::Compute(const std::vector<cv::Mat>& descrs1, const std::vector<cv::Mat>& descrs2) {
    // Ensure the input vectors are not empty and have the same size.
    PT_CHECK(descrs1.size() != 0);
    PT_CHECK(descrs1.size() == descrs2.size());

    // Initialize the distances vector with default value 1.0f.
    std::vector<float> distances(descrs1.size(), 1.f);
    for (size_t i = 0; i < descrs1.size(); i++) {
        // Compute the distance for each pair of descriptors.
        distances.at(i) = Compute(descrs1.at(i), descrs2.at(i));
    }

    // Return the vector of distances.
    return distances;
}

// Compute the template matching distance between two descriptors.
// A high return value indicates a poor match, while a low return value indicates a good match.
float MatchTemplateDistance::Compute(const cv::Mat& descr1, const cv::Mat& descr2) {
    // Ensure the descriptors are not empty and match in size and type.
    PT_CHECK(!descr1.empty() && !descr2.empty());
    PT_CHECK_EQ(descr1.size(), descr2.size());
    PT_CHECK_EQ(descr1.type(), descr2.type());

    // Perform template matching.
    cv::Mat res;
    cv::matchTemplate(descr1, descr2, res, type_);
    PT_CHECK(res.size() == cv::Size(1, 1));

    // Retrieve the matching distance.
    float dist = res.at<float>(0, 0);

    // Return the scaled and offset distance.
    // A high value indicates a poor match, while a low value indicates a good match.
    return scale_ * dist + offset_;
}

// Compute the template matching distances between pairs of descriptors from two vectors.
// Returns a vector of distances.
std::vector<float> MatchTemplateDistance::Compute(const std::vector<cv::Mat>& descrs1,
                                                  const std::vector<cv::Mat>& descrs2) {
    // Initialize the result vector.
    std::vector<float> result;
    for (size_t i = 0; i < descrs1.size(); i++) {
        // Compute the distance for each pair of descriptors.
        result.push_back(Compute(descrs1[i], descrs2[i]));
    }
    // Return the vector of distances.
    return result;
}
