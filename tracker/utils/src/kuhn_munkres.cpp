// Copyright (C) 2018-2023 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
//

#include <algorithm>
#include <limits>
#include <vector>
#include <iostream> // Added to support std::cerr

#include "../include/utils/kuhn_munkres.hpp"

KuhnMunkres::KuhnMunkres(bool greedy) : n_(), greedy_(greedy) {}

// This function solves the assignment problem using the Kuhn-Munkres algorithm, also known as the Hungarian algorithm.
// It takes a dissimilarity matrix as input and returns a vector of indices indicating the optimal assignment.
std::vector<size_t> KuhnMunkres::Solve(const cv::Mat& dissimilarity_matrix) {
    // Ensure the input matrix is of type CV_32F (32-bit float) for compatibility.
    CV_Assert(dissimilarity_matrix.type() == CV_32F);
    double min_val;
    // Find the minimum value in the dissimilarity matrix. This is not used directly but is part of the algorithm's requirements.
    cv::minMaxLoc(dissimilarity_matrix, &min_val);

    // Determine the size of the problem (n_) by finding the maximum dimension of the input matrix.
    n_ = std::max(dissimilarity_matrix.rows, dissimilarity_matrix.cols);
    // Initialize the working dissimilarity matrix (dm_) with zeros. This matrix will be square with size n_.
    dm_ = cv::Mat(n_, n_, CV_32F, cv::Scalar(0));
    // Initialize the marked matrix with zeros. This matrix keeps track of the assignments and is also square with size n_.
    marked_ = cv::Mat(n_, n_, CV_8S, cv::Scalar(0));
    // Initialize a vector to store points, with a size twice that of n_.
    points_ = std::vector<cv::Point>(n_ * 2);

    // Copy the input dissimilarity matrix into the top-left corner of the working dissimilarity matrix (dm_).
    dissimilarity_matrix.copyTo(dm_(
            cv::Rect(0, 0, dissimilarity_matrix.cols, dissimilarity_matrix.rows)));

    // Initialize vectors to keep track of visited rows and columns during the algorithm's execution, initially setting all to not visited (0).
    is_row_visited_ = std::vector<int>(n_, 0);
    is_col_visited_ = std::vector<int>(n_, 0);

    // Run the main part of the Kuhn-Munkres algorithm to find the optimal assignment.
    Run();

    // Initialize the results vector with a size equal to the number of rows in the input matrix, filling it with -1 to indicate no assignment.
    std::vector<size_t> results(dissimilarity_matrix.rows, -1);
    // Iterate over each row in the marked matrix to find the assignments.
    for (int i = 0; i < dissimilarity_matrix.rows; i++) {
        // Get a pointer to the current row in the marked matrix.
        const auto ptr = marked_.ptr<char>(i);
        // Iterate over each column in the current row.
        for (int j = 0; j < dissimilarity_matrix.cols; j++) {
            // If the current cell is marked with a star (kStar), it indicates an assignment.
            if (ptr[j] == kStar) {
                // Record the assignment in the results vector. The index in the results vector corresponds to the row (assignment from),
                // and the value at that index corresponds to the column (assignment to).
                results[i] = (size_t)j;
            }
            // For a cell not to be marked with a star, indicating no assignment, it means that during the algorithm's execution,
            // no optimal assignment was found for that specific cell. This could occur if the cell's value in the dissimilarity matrix
            // does not lead to the lowest possible total cost when compared with other potential assignments. Specifically, if the algorithm
            // determines that assigning a row to a column (or vice versa) would result in a higher total cost than other possible assignments,
            // or if all suitable assignments for a row or column have already been made, the cell would remain unmarked. In essence, for a cell
            // to remain unmarked, its value would have to be such that including it in the set of assignments would not contribute to the
            // minimization of the total cost, or all preferable assignments have been exhausted.

            // Regarding new detections entering the frame, it is common for a cell to remain unmarked in scenarios where there is a mismatch
            // in the number of tracks and detections. For example, if there is one active track and another detection appears, the algorithm
            // may not immediately find an optimal assignment for the new detection if it does not closely match any existing tracks, leading
            // to an unmarked cell. This situation is not rare and can frequently occur in dynamic environments where new objects enter the scene,
            // or existing objects leave the scene, creating discrepancies between the number of tracks and detections. The algorithm's ability
            // to adapt and find new assignments as the scene evolves is crucial for maintaining accurate tracking over time.
        }
    }
    // Return the vector of assignments. If there is no assignment for a particular row, its value remains -1.
    return results;
}

void KuhnMunkres::TrySimpleCase() {
    auto is_row_visited = std::vector<int>(n_, 0);
    auto is_col_visited = std::vector<int>(n_, 0);

    for (int row = 0; row < n_; row++) {
        auto ptr = dm_.ptr<float>(row);
        auto marked_ptr = marked_.ptr<char>(row);
        auto min_val = *std::min_element(ptr, ptr + n_);
        for (int col = 0; col < n_; col++) {
            ptr[col] -= min_val;
            if (ptr[col] == 0 && !is_col_visited[col] && !is_row_visited[row]) {
                marked_ptr[col] = kStar;
                is_col_visited[col] = 1;
                is_row_visited[row] = 1;
            }
        }
    }
}

bool KuhnMunkres::CheckIfOptimumIsFound() {
    int count = 0;
    for (int i = 0; i < n_; i++) {
        const auto marked_ptr = marked_.ptr<char>(i);
        for (int j = 0; j < n_; j++) {
            if (marked_ptr[j] == kStar) {
                is_col_visited_[j] = 1;
                count++;
            }
        }
    }

    return count >= n_;
}

cv::Point KuhnMunkres::FindUncoveredMinValPos() {
    auto min_val = std::numeric_limits<float>::max();
    cv::Point min_val_pos(-1, -1);
    for (int i = 0; i < n_; i++) {
        if (!is_row_visited_[i]) {
            auto dm_ptr = dm_.ptr<float>(i);
            for (int j = 0; j < n_; j++) {
                if (!is_col_visited_[j] && dm_ptr[j] < min_val) {
                    min_val = dm_ptr[j];
                    min_val_pos = cv::Point(j, i);
                }
            }
        }
    }
    return min_val_pos;
}

void KuhnMunkres::UpdateDissimilarityMatrix(float val) {
    for (int i = 0; i < n_; i++) {
        auto dm_ptr = dm_.ptr<float>(i);
        for (int j = 0; j < n_; j++) {
            if (is_row_visited_[i]) dm_ptr[j] += val;
            if (!is_col_visited_[j]) dm_ptr[j] -= val;
        }
    }
}

int KuhnMunkres::FindInRow(int row, int what) {
    for (int j = 0; j < n_; j++) {
        if (marked_.at<char>(row, j) == what) {
            return j;
        }
    }
    return -1;
}

int KuhnMunkres::FindInCol(int col, int what) {
    for (int i = 0; i < n_; i++) {
        if (marked_.at<char>(i, col) == what) {
            return i;
        }
    }
    return -1;
}

void KuhnMunkres::Run() {
    TrySimpleCase();
    if (greedy_)
        return;
    while (!CheckIfOptimumIsFound()) {
        while (true) {
            auto point = FindUncoveredMinValPos();
            auto min_val = dm_.at<float>(point.y, point.x);
            if (min_val > 0) {
                UpdateDissimilarityMatrix(min_val);
            } else {
                marked_.at<char>(point.y, point.x) = kPrime;
                int col = FindInRow(point.y, kStar);
                if (col >= 0) {
                    is_row_visited_[point.y] = 1;
                    is_col_visited_[col] = 0;
                } else {
                    int count = 0;
                    points_[count] = point;

                    while (true) {
                        int row = FindInCol(points_[count].x, kStar);
                        if (row >= 0) {
                            count++;
                            points_[count] = cv::Point(points_[count - 1].x, row);
                            int col = FindInRow(points_[count].y, kPrime);
                            count++;
                            points_[count] = cv::Point(col, points_[count - 1].y);
                        } else {
                            break;
                        }
                    }

                    for (int i = 0; i < count + 1; i++) {
                        auto& mark = marked_.at<char>(points_[i].y, points_[i].x);
                        mark = mark == kStar ? 0 : kStar;
                    }

                    is_row_visited_ = std::vector<int>(n_, 0);
                    is_col_visited_ = std::vector<int>(n_, 0);

                    marked_.setTo(0, marked_ == kPrime);
                    break;
                }
            }
        }
    }
}
