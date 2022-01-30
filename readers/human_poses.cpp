// Copyright (C) 2018-2019 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
//

//#define VERBOSE

#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include "human_poses.h"
#include "../utils/utils.h"

namespace moetsi::ssp {
namespace human_pose_estimation {

HumanPose::HumanPose(const std::vector<cv::Point3f>& keypoints,
                     const float& score)
    : keypoints(keypoints),
      score(score) {}
} // namespace human_pose_estimation


namespace human_pose_estimation {
static void resizeFeatureMaps(std::vector<cv::Mat>& featureMaps, int upsampleRatio) {
    for (auto& featureMap : featureMaps) {
        cv::resize(featureMap, featureMap, cv::Size(),
                   upsampleRatio, upsampleRatio, cv::INTER_CUBIC);
    }
}

class FindPeaksBody: public cv::ParallelLoopBody {
public:
    FindPeaksBody(const std::vector<cv::Mat>& heatMaps, float minPeaksDistance,
                  std::vector<std::vector<Peak> >& peaksFromHeatMap)
        : heatMaps(heatMaps),
          minPeaksDistance(minPeaksDistance),
          peaksFromHeatMap(peaksFromHeatMap) {}

    virtual void operator()(const cv::Range& range) const {
        for (int i = range.start; i < range.end; i++) {
            findPeaks(heatMaps, minPeaksDistance, peaksFromHeatMap, i);
        }
    }

private:
    const std::vector<cv::Mat>& heatMaps;
    float minPeaksDistance;
    std::vector<std::vector<Peak> >& peaksFromHeatMap;
};

std::vector<HumanPose> extractPoses(
        std::vector<cv::Mat>& heatMaps,
        std::vector<cv::Mat>& pafs,
        int upsampleRatio) {
    resizeFeatureMaps(heatMaps, upsampleRatio);
    resizeFeatureMaps(pafs, upsampleRatio);
    std::vector<std::vector<Peak> > peaksFromHeatMap(heatMaps.size());
    float minPeaksDistance = 6.0f; // 3.0f ??
    FindPeaksBody findPeaksBody(heatMaps, minPeaksDistance, peaksFromHeatMap);
    cv::parallel_for_(cv::Range(0, static_cast<int>(heatMaps.size())),
                      findPeaksBody);
    int peaksBefore = 0;
    for (size_t heatmapId = 1; heatmapId < heatMaps.size(); heatmapId++) {
        peaksBefore += static_cast<int>(peaksFromHeatMap[heatmapId - 1].size());
        for (auto& peak : peaksFromHeatMap[heatmapId]) {
            peak.id += peaksBefore;
        }
    }
    int keypointsNumber = 18;
    float midPointsScoreThreshold = 0.05f;
    float foundMidPointsRatioThreshold = 0.8f;
    int minJointsNumber = 3;
    float minSubsetScore = 0.2f;
    std::vector<HumanPose> poses = groupPeaksToPoses(
                peaksFromHeatMap, pafs, keypointsNumber, midPointsScoreThreshold,
                foundMidPointsRatioThreshold, minJointsNumber, minSubsetScore);
    return poses;
}

} // namespace human_pose_estimation
// Copyright (C) 2018-2019 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
//

#include <algorithm>
#include <utility>
#include <vector>

namespace human_pose_estimation {
Peak::Peak(const int id, const cv::Point2f& pos, const float score)
    : id(id),
      pos(pos),
      score(score) {}

HumanPoseByPeaksIndices::HumanPoseByPeaksIndices(const int keypointsNumber)
    : peaksIndices(std::vector<int>(keypointsNumber, -1)),
      nJoints(0),
      score(0.0f) {}

TwoJointsConnection::TwoJointsConnection(const int firstJointIdx,
                                         const int secondJointIdx,
                                         const float score)
    : firstJointIdx(firstJointIdx),
      secondJointIdx(secondJointIdx),
      score(score) {}

void findPeaks(const std::vector<cv::Mat>& heatMaps,
               const float minPeaksDistance,
               std::vector<std::vector<Peak> >& allPeaks,
               int heatMapId) {
    const float threshold = 0.1f;
    std::vector<cv::Point> peaks;
    const cv::Mat& heatMap = heatMaps[heatMapId];
    const float* heatMapData = heatMap.ptr<float>();
    size_t heatMapStep = heatMap.step1();
    for (int y = -1; y < heatMap.rows + 1; y++) {
        for (int x = -1; x < heatMap.cols + 1; x++) {
            float val = 0;
            if (x >= 0
                    && y >= 0
                    && x < heatMap.cols
                    && y < heatMap.rows) {
                val = heatMapData[y * heatMapStep + x];
                val = val >= threshold ? val : 0;
            }

            float left_val = 0;
            if (y >= 0
                    && x < (heatMap.cols - 1)
                    && y < heatMap.rows) {
                left_val = heatMapData[y * heatMapStep + x + 1];
                left_val = left_val >= threshold ? left_val : 0;
            }

            float right_val = 0;
            if (x > 0
                    && y >= 0
                    && y < heatMap.rows) {
                right_val = heatMapData[y * heatMapStep + x - 1];
                right_val = right_val >= threshold ? right_val : 0;
            }

            float top_val = 0;
            if (x >= 0
                    && x < heatMap.cols
                    && y < (heatMap.rows - 1)) {
                top_val = heatMapData[(y + 1) * heatMapStep + x];
                top_val = top_val >= threshold ? top_val : 0;
            }

            float bottom_val = 0;
            if (x >= 0
                    && y > 0
                    && x < heatMap.cols) {
                bottom_val = heatMapData[(y - 1) * heatMapStep + x];
                bottom_val = bottom_val >= threshold ? bottom_val : 0;
            }

            if ((val > left_val)
                    && (val > right_val)
                    && (val > top_val)
                    && (val > bottom_val)) {
                //if (x+3 < heatMap.cols) { // TODO restore
                    peaks.push_back(cv::Point(x, y));
                //}
            }
        }
    }
    std::sort(peaks.begin(), peaks.end(), [](const cv::Point& a, const cv::Point& b) {
        return a.x < b.x;
    });
    std::vector<bool> isActualPeak(peaks.size(), true);
    int peakCounter = 0;
    std::vector<Peak>& peaksWithScoreAndID = allPeaks[heatMapId];
    for (size_t i = 0; i < peaks.size(); i++) {
        if (isActualPeak[i]) {
            for (size_t j = i + 1; j < peaks.size(); j++) {
                if (sqrt((peaks[i].x - peaks[j].x) * (peaks[i].x - peaks[j].x) +
                         (peaks[i].y - peaks[j].y) * (peaks[i].y - peaks[j].y)) < minPeaksDistance) {
                    isActualPeak[j] = false;
                }
            }
            peaksWithScoreAndID.push_back(Peak(peakCounter++, peaks[i], heatMap.at<float>(peaks[i])));
        }
    }
}

std::vector<HumanPose> groupPeaksToPoses(const std::vector<std::vector<Peak> >& allPeaks,
                                         const std::vector<cv::Mat>& pafs,
                                         const size_t keypointsNumber,
                                         const float midPointsScoreThreshold,
                                         const float foundMidPointsRatioThreshold,
                                         const int minJointsNumber,
                                         const float minSubsetScore) {
    const std::vector<std::pair<int, int> > limbIdsHeatmap = {
        {2, 3}, {2, 6}, {3, 4}, {4, 5}, {6, 7}, {7, 8}, {2, 9}, {9, 10}, {10, 11}, {2, 12}, {12, 13}, {13, 14},
        {2, 1}, {1, 15}, {15, 17}, {1, 16}, {16, 18}, {3, 17}, {6, 18}
    };
    const std::vector<std::pair<int, int> > limbIdsPaf = {
        {31, 32}, {39, 40}, {33, 34}, {35, 36}, {41, 42}, {43, 44}, {19, 20}, {21, 22}, {23, 24}, {25, 26},
        {27, 28}, {29, 30}, {47, 48}, {49, 50}, {53, 54}, {51, 52}, {55, 56}, {37, 38}, {45, 46}
    };

    std::vector<Peak> candidates;
    for (const auto& peaks : allPeaks) {
         candidates.insert(candidates.end(), peaks.begin(), peaks.end());
    }
    std::vector<HumanPoseByPeaksIndices> subset(0, HumanPoseByPeaksIndices(static_cast<int>(keypointsNumber)));
    for (size_t k = 0; k < limbIdsPaf.size(); k++) {
        std::vector<TwoJointsConnection> connections;
        const int mapIdxOffset = static_cast<int>(keypointsNumber) + 1;
        std::pair<cv::Mat, cv::Mat> scoreMid = { pafs[limbIdsPaf[k].first - mapIdxOffset],
                                                 pafs[limbIdsPaf[k].second - mapIdxOffset] };
        const int idxJointA = limbIdsHeatmap[k].first - 1;
        const int idxJointB = limbIdsHeatmap[k].second - 1;
        const std::vector<Peak>& candA = allPeaks[idxJointA];
        const std::vector<Peak>& candB = allPeaks[idxJointB];
        const size_t nJointsA = candA.size();
        const size_t nJointsB = candB.size();
        if (nJointsA == 0
                && nJointsB == 0) {
            continue;
        } else if (nJointsA == 0) {
            for (size_t i = 0; i < nJointsB; i++) {
                int num = 0;
                for (size_t j = 0; j < subset.size(); j++) {
                    if (subset[j].peaksIndices[idxJointB] == candB[i].id) {
                        num++;
                        continue;
                    }
                }
                if (num == 0) {
                    HumanPoseByPeaksIndices personKeypoints(static_cast<int>(keypointsNumber));
                    personKeypoints.peaksIndices[idxJointB] = candB[i].id;
                    personKeypoints.nJoints = 1;
                    personKeypoints.score = candB[i].score;
                    subset.push_back(personKeypoints);
                }
            }
            continue;
        } else if (nJointsB == 0) {
            for (size_t i = 0; i < nJointsA; i++) {
                int num = 0;
                for (size_t j = 0; j < subset.size(); j++) {
                    if (subset[j].peaksIndices[idxJointA] == candA[i].id) {
                        num++;
                        continue;
                    }
                }
                if (num == 0) {
                    HumanPoseByPeaksIndices personKeypoints(static_cast<int>(keypointsNumber));
                    personKeypoints.peaksIndices[idxJointA] = candA[i].id;
                    personKeypoints.nJoints = 1;
                    personKeypoints.score = candA[i].score;
                    subset.push_back(personKeypoints);
                }
            }
            continue;
        }

        std::vector<TwoJointsConnection> tempJointConnections;
        for (size_t i = 0; i < nJointsA; i++) {
            for (size_t j = 0; j < nJointsB; j++) {
                cv::Point2f pt = candA[i].pos * 0.5 + candB[j].pos * 0.5;
                cv::Point mid = cv::Point(cvRound(pt.x), cvRound(pt.y));
                cv::Point2f vec = candB[j].pos - candA[i].pos;
                double norm_vec = cv::norm(vec);
                if (norm_vec == 0) {
                    continue;
                }
                vec /= norm_vec;
                float score = vec.x * scoreMid.first.at<float>(mid) + vec.y * scoreMid.second.at<float>(mid);
                int height_n  = pafs[0].rows / 2;
                float suc_ratio = 0.0f;
                float mid_score = 0.0f;
                const int mid_num = 10;
                const float scoreThreshold = -100.0f;
                if (score > scoreThreshold) {
                    float p_sum = 0;
                    int p_count = 0;
                    cv::Size2f step((candB[j].pos.x - candA[i].pos.x)/(mid_num - 1),
                                    (candB[j].pos.y - candA[i].pos.y)/(mid_num - 1));
                    for (int n = 0; n < mid_num; n++) {
                        cv::Point midPoint(cvRound(candA[i].pos.x + n * step.width),
                                           cvRound(candA[i].pos.y + n * step.height));
                        cv::Point2f pred(scoreMid.first.at<float>(midPoint),
                                         scoreMid.second.at<float>(midPoint));
                        score = vec.x * pred.x + vec.y * pred.y;
                        if (score > midPointsScoreThreshold) {
                            p_sum += score;
                            p_count++;
                        }
                    }
                    suc_ratio = static_cast<float>(p_count / mid_num);
                    float ratio = p_count > 0 ? p_sum / p_count : 0.0f;
                    mid_score = ratio + static_cast<float>(std::min(height_n / norm_vec - 1, 0.0));
                }
                if (mid_score > 0
                        && suc_ratio > foundMidPointsRatioThreshold) {
                    tempJointConnections.push_back(TwoJointsConnection(static_cast<int>(i), static_cast<int>(j), mid_score));
                }
            }
        }
        if (!tempJointConnections.empty()) {
            std::sort(tempJointConnections.begin(), tempJointConnections.end(),
                      [](const TwoJointsConnection& a,
                         const TwoJointsConnection& b) {
                return (a.score > b.score);
            });
        }
        size_t num_limbs = std::min(nJointsA, nJointsB);
        size_t cnt = 0;
        std::vector<int> occurA(nJointsA, 0);
        std::vector<int> occurB(nJointsB, 0);
        for (size_t row = 0; row < tempJointConnections.size(); row++) {
            if (cnt == num_limbs) {
                break;
            }
            const int& indexA = tempJointConnections[row].firstJointIdx;
            const int& indexB = tempJointConnections[row].secondJointIdx;
            const float& score = tempJointConnections[row].score;
            if (occurA[indexA] == 0
                    && occurB[indexB] == 0) {
                connections.push_back(TwoJointsConnection(candA[indexA].id, candB[indexB].id, score));
                cnt++;
                occurA[indexA] = 1;
                occurB[indexB] = 1;
            }
        }
        if (connections.empty()) {
            continue;
        }

        bool extraJointConnections = (k == 17 || k == 18);
        if (k == 0) {
            subset = std::vector<HumanPoseByPeaksIndices>(
                        connections.size(), HumanPoseByPeaksIndices(static_cast<int>(keypointsNumber)));
            for (size_t i = 0; i < connections.size(); i++) {
                const int& indexA = connections[i].firstJointIdx;
                const int& indexB = connections[i].secondJointIdx;
                subset[i].peaksIndices[idxJointA] = indexA;
                subset[i].peaksIndices[idxJointB] = indexB;
                subset[i].nJoints = 2;
                subset[i].score = candidates[indexA].score + candidates[indexB].score + connections[i].score;
            }
        } else if (extraJointConnections) {
            for (size_t i = 0; i < connections.size(); i++) {
                const int& indexA = connections[i].firstJointIdx;
                const int& indexB = connections[i].secondJointIdx;
                for (size_t j = 0; j < subset.size(); j++) {
                    if (subset[j].peaksIndices[idxJointA] == indexA
                            && subset[j].peaksIndices[idxJointB] == -1) {
                        subset[j].peaksIndices[idxJointB] = indexB;
                    } else if (subset[j].peaksIndices[idxJointB] == indexB
                                && subset[j].peaksIndices[idxJointA] == -1) {
                        subset[j].peaksIndices[idxJointA] = indexA;
                    }
                }
            }
            continue;
        } else {
            for (size_t i = 0; i < connections.size(); i++) {
                const int& indexA = connections[i].firstJointIdx;
                const int& indexB = connections[i].secondJointIdx;
                bool num = false;
                for (size_t j = 0; j < subset.size(); j++) {
                    if (subset[j].peaksIndices[idxJointA] == indexA) {
                        subset[j].peaksIndices[idxJointB] = indexB;
                        subset[j].nJoints++;
                        subset[j].score += candidates[indexB].score + connections[i].score;
                        num = true;
                    }
                }
                if (!num) {
                    HumanPoseByPeaksIndices hpWithScore(static_cast<int>(keypointsNumber));
                    hpWithScore.peaksIndices[idxJointA] = indexA;
                    hpWithScore.peaksIndices[idxJointB] = indexB;
                    hpWithScore.nJoints = 2;
                    hpWithScore.score = candidates[indexA].score + candidates[indexB].score + connections[i].score;
                    subset.push_back(hpWithScore);
                }
            }
        }
    }
    std::vector<HumanPose> poses;
    for (const auto& subsetI : subset) {
        if (subsetI.nJoints < minJointsNumber
                || subsetI.score / subsetI.nJoints < minSubsetScore) {
            continue;
        }
        int position = -1;
        HumanPose pose(std::vector<cv::Point3f>(keypointsNumber, cv::Point3f(-1.0f, -1.0f, -1.0f)),
                       subsetI.score * std::max(0, subsetI.nJoints - 1));
        for (const auto& peakIdx : subsetI.peaksIndices) {
            position++;
            if (peakIdx >= 0) {

#ifdef VERBOSE
                std::cerr << "peakIdx = " << peakIdx << " " << candidates[peakIdx].pos.x << " " << candidates[peakIdx].pos.y << " " << candidates[peakIdx].score << std::endl << std::flush;
#endif
                pose.keypoints[position].x = candidates[peakIdx].pos.x; // + 0.5f; // TODO why + 0.5f ??
                pose.keypoints[position].y = candidates[peakIdx].pos.y; // + 0.5f;
                pose.keypoints[position].z = candidates[peakIdx].score;
            }
        }
        poses.push_back(pose);
    }

//exit(-1);
    return poses;
}
} // namespace human_pose_estimation

#include <iostream>

// #define VERBOSE
namespace human_pose_estimation {
    int AVG_PERSON_HEIGHT = 180;

    // pelvis (body center) is missing, id == 2
    std::vector<int> map_id_to_panoptic = { 1, 0, 9, 10, 11, 3, 4, 5, 12, 13, 14, 6, 7, 8, 15, 16, 17, 18 };

    std::vector<cv::Point3i> limbs = { cv::Point3i(18, 17, 1),
          cv::Point3i(16, 15, 1),
          cv::Point3i(5, 4, 3),
          cv::Point3i(8, 7, 6),
          cv::Point3i(11, 10, 9),
          cv::Point3i(14, 13, 12) };

    inline std::vector<cv::Mat> slices(const cv::Mat & e) {
        std::vector<cv::Mat> rv;
        rv.resize(e.size[0]);
        for (int i = 0; i< e.size[0]; ++i) {
            rv[i] = cv::Mat(e.size[1], e.size[2], CV_32F);
            for (int j= 0; j < e.size[1]; ++j) {
                for (int k = 0; k < e.size[2]; ++k) {
                    rv[i].at<float>(j,k) = e.at<float>(i,j,k);
                }
            }
        }
        return rv;
    }

    root_relative_poses get_root_relative_poses(const cv::Mat &features, const cv::Mat &heatmap, const cv::Mat &paf_map) {
#ifdef VERBOSE
        std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
#endif
        int upsample_ratio = 4;
        
        // (19, 32, 56) heatmap[0:-1] shape =  (18, 32, 56)
        int dims[] { heatmap.size[0]-1, heatmap.size[1], heatmap.size[2] };
        cv::Mat heatmap_0m1(3, dims, CV_32F);

#ifdef VERBOSE
        std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
#endif

        for (int i=0; i< dims[0]; ++i) {
            for (int j=0; j<dims[1]; ++j) {
                for(int k=0; k<dims[2]; ++k) {
                    heatmap_0m1.at<float>(i,j,k) = heatmap.at<float>(i,j,k);
                }
            }
        }

#ifdef VERBOSE        
        std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
#endif

        auto hm = slices(heatmap_0m1);

#ifdef VERBOSE
        std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
#endif
        auto spm = slices(paf_map);

#ifdef VERBOSE
        std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
#endif

        std::vector<HumanPose> found_poses = extractPoses(hm, //heatmap[0:-1]
                                                        spm, upsample_ratio); // [0] HERE ~~~

#ifdef VERBOSE
        std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;

        for (int i=0; i< int(found_poses.size()); ++i) {
            std::cerr << i << "#" << found_poses[i].score << std::endl;
            for (int j=0; j < int(found_poses[i].keypoints.size()); ++j) {
                std::cerr << found_poses[i].keypoints[j].x << "," <<
                            found_poses[i].keypoints[j].y << "," << 
                            found_poses[i].keypoints[j].z << std::endl;
            }
            std::cerr << std::flush;
        }
#endif

        //  # scale coordinates to features space
        //    found_poses[:, 0:-1:3] /= upsample_ratio
        //    found_poses[:, 1:-1:3] /= upsample_ratio
        // => /=4 x and y!!!

        for (int i=0; i< int(found_poses.size()); ++i) {
#ifdef VERBOSE            
            std::cerr << i << "#" << found_poses[i].score << std::endl;
#endif
            for (int j=0; j < int(found_poses[i].keypoints.size()); ++j) {
                found_poses[i].keypoints[j].x /= upsample_ratio;
                found_poses[i].keypoints[j].y /= upsample_ratio;
            }
        }        

        std::vector<std::vector<float>> poses_2d;
        int num_kpt_panoptic = 19;
        int num_kpt = 18;

        for (int pose_id=0; pose_id< int(found_poses.size()); ++pose_id) {
#ifdef VERBOSE
            std::cerr << pose_id << "#" << found_poses[pose_id].score << std::endl;
#endif
            auto cmp = found_poses[pose_id].keypoints[1].z;
            if (cmp == -1) {
                continue; // skip if no neck!
            }
            // pose_2d = np.ones(num_kpt_panoptic * 3 + 1, dtype=np.float32) * -1  # +1 for pose confidence
            std::vector<float> pose_2d;
            pose_2d.resize(num_kpt_panoptic * 3 + 1);
            for (int j=0; j< int(pose_2d.size()); ++j) pose_2d[j] = -1;

            for (int kpt_id=0; kpt_id < num_kpt; ++kpt_id) {

#ifdef VERBOSE               
                std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
#endif

                if (found_poses[pose_id].keypoints[kpt_id].z != -1) {
                    float x_2d = found_poses[pose_id].keypoints[kpt_id].x;
                    float y_2d = found_poses[pose_id].keypoints[kpt_id].y;
                    float conf = found_poses[pose_id].keypoints[kpt_id].z;
                    pose_2d[map_id_to_panoptic[kpt_id] * 3] = x_2d;
                    pose_2d[map_id_to_panoptic[kpt_id] * 3 + 1] = y_2d;
                    pose_2d[map_id_to_panoptic[kpt_id] * 3 + 2] = conf;
                }
            }
            
            *pose_2d.rbegin() = found_poses[pose_id].score; 

#ifdef VERBOSE
            {
                std::cerr << "pose: " << pose_id << std::endl << std::flush;
                for (int j=0; j< int(pose_2d.size()); ++j) {
                    std::cerr << pose_2d[j] << ","; 
                }
                std::cerr << std::endl << std::flush;
            }
#endif

            poses_2d.push_back(pose_2d);
        }   

        float keypoint_treshold = 0.1;
        // poses_3d = np.ones((len(poses_2d), num_kpt_panoptic * 4), dtype=np.float32) * -1
        std::vector<std::vector<float>> poses_3d;    

#ifdef VERBOSE
        std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
#endif

        for (int pose_id=0; pose_id < int(poses_2d.size()); ++pose_id) {

            int neck_2d_x = 0;
            int neck_2d_y = 0;
            if (poses_2d[pose_id][2] > keypoint_treshold) {
                neck_2d_x = int(poses_2d[pose_id][0]);
                neck_2d_y = int(poses_2d[pose_id][1]);
                
                std::vector<float> pose_3d;
                pose_3d.resize(num_kpt_panoptic * 4);
                for (int j=0; j< int(pose_3d.size()); ++j) pose_3d[j] = -1;

    #ifdef VERBOSE
                std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    #endif

                for (int kpt_id =0; kpt_id < num_kpt_panoptic; ++kpt_id) {
                    int map_3d_idx = kpt_id * 3;
                    pose_3d[kpt_id *4] = features.at<float>(map_3d_idx, neck_2d_y, neck_2d_x) * AVG_PERSON_HEIGHT;

    #ifdef VERBOSE
                    std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    #endif

                    pose_3d[kpt_id *4 + 1] = features.at<float>(map_3d_idx+1, neck_2d_y, neck_2d_x) * AVG_PERSON_HEIGHT;

    #ifdef VERBOSE
                    std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    #endif

                    pose_3d[kpt_id *4 + 2] = features.at<float>(map_3d_idx+2, neck_2d_y, neck_2d_x) * AVG_PERSON_HEIGHT;

    #ifdef VERBOSE
                    std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    #endif

                    pose_3d[kpt_id * 4 + 3] = poses_2d[pose_id][kpt_id * 3 + 2];
                }

    #ifdef VERBOSE
                {
                    std::cerr << "3d pose/1 : " << pose_id << std::endl;
                    for (int j=0; j< int(pose_3d.size()); ++j) {
                        std::cerr << pose_3d[j] << ", ";
                    }
                    std::cerr << std::endl << std::flush;
                }
    #endif

                for (int limb = 0; limb < limbs.size(); ++limb) {
                    int kpt_id_from = 0;

                    bool is_not_break = true;

    #ifdef VERBOSE
                    std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    #endif
                    if (is_not_break) {
                        kpt_id_from = limbs[limb].x;

    #ifdef VERBOSE
                        std::cerr << "limb  " << limb << " " << kpt_id_from << std::endl;
                        std::cerr << (poses_2d[pose_id][kpt_id_from * 3 + 2]) << " " << keypoint_treshold << " " << int(poses_2d[pose_id][kpt_id_from * 3 + 2] > keypoint_treshold) << std::endl << std::flush;
    #endif

                        if (poses_2d[pose_id][kpt_id_from * 3 + 2] > keypoint_treshold)
                        {
                            {
                                int kpt_id_where = 0;

                                {
                                    kpt_id_where = limbs[limb].x;
                                    int map_3d_id = kpt_id_where * 3;
                                    int kpt_from_2d_x = int(poses_2d[pose_id][kpt_id_from *3]);
                                    int kpt_from_2d_y = int(poses_2d[pose_id][kpt_id_from *3 + 1]);

    #ifdef VERBOSE
                                    std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
                                    std::cerr << "<< " << kpt_id_from << " " << kpt_id_where << " " << kpt_from_2d_x << "," << kpt_from_2d_y << std::endl << std::flush; 
    #endif

                                    //poses_3d[pose_id][kpt_id_where * 4] = map_3d[0, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HEIGHT
                                    pose_3d[kpt_id_where * 4] = features.at<float>(map_3d_id, kpt_from_2d_y, kpt_from_2d_x) * AVG_PERSON_HEIGHT;
                                    //poses_3d[pose_id][kpt_id_where * 4 + 1] = map_3d[1, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HEIGHT
                                    pose_3d[kpt_id_where * 4+1] = features.at<float>(map_3d_id+1, kpt_from_2d_y, kpt_from_2d_x) * AVG_PERSON_HEIGHT;
                                    //poses_3d[pose_id][kpt_id_where * 4 + 2] = map_3d[2, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HEIGHT
                                    pose_3d[kpt_id_where * 4+2] = features.at<float>(map_3d_id+2, kpt_from_2d_y, kpt_from_2d_x) * AVG_PERSON_HEIGHT;
                                }
                                is_not_break = false;
                            }

                            {
                                int kpt_id_where = 0;
                                {
                                    kpt_id_where = limbs[limb].y;
                                    int map_3d_id = kpt_id_where * 3;
                                    int kpt_from_2d_x = int(poses_2d[pose_id][kpt_id_from *3]);
                                    int kpt_from_2d_y = int(poses_2d[pose_id][kpt_id_from *3 + 1]);

    #ifdef VERBOSE
                                    std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
                                    std::cerr << "<< " << kpt_id_from << " " << kpt_id_where << " " << kpt_from_2d_x << "," << kpt_from_2d_y << std::endl << std::flush; 
    #endif

                                    //poses_3d[pose_id][kpt_id_where * 4] = map_3d[0, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HEIGHT
                                    pose_3d[kpt_id_where * 4] = features.at<float>(map_3d_id, kpt_from_2d_y, kpt_from_2d_x) * AVG_PERSON_HEIGHT;
                                    //poses_3d[pose_id][kpt_id_where * 4 + 1] = map_3d[1, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HEIGHT
                                    pose_3d[kpt_id_where * 4+1] = features.at<float>(map_3d_id+1, kpt_from_2d_y, kpt_from_2d_x) * AVG_PERSON_HEIGHT;
                                    //poses_3d[pose_id][kpt_id_where * 4 + 2] = map_3d[2, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HEIGHT
                                    pose_3d[kpt_id_where * 4+2] = features.at<float>(map_3d_id+2, kpt_from_2d_y, kpt_from_2d_x) * AVG_PERSON_HEIGHT;
                                }
                                is_not_break = false;
                            }

                            {
                                int kpt_id_where = 0;
                                {
                                    kpt_id_where = limbs[limb].z;
                                    int map_3d_id = kpt_id_where * 3;
                                    int kpt_from_2d_x = int(poses_2d[pose_id][kpt_id_from *3]);
                                    int kpt_from_2d_y = int(poses_2d[pose_id][kpt_id_from *3 + 1]);

    #ifdef VERBOSE
                                    std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
                                    std::cerr << "<< " << kpt_id_from << " " << kpt_id_where << " " << kpt_from_2d_x << "," << kpt_from_2d_y << std::endl << std::flush; 
    #endif

                                    //poses_3d[pose_id][kpt_id_where * 4] = map_3d[0, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HEIGHT
                                    pose_3d[kpt_id_where * 4] = features.at<float>(map_3d_id, kpt_from_2d_y, kpt_from_2d_x) * AVG_PERSON_HEIGHT;
                                    //poses_3d[pose_id][kpt_id_where * 4 + 1] = map_3d[1, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HEIGHT
                                    pose_3d[kpt_id_where * 4+1] = features.at<float>(map_3d_id+1, kpt_from_2d_y, kpt_from_2d_x) * AVG_PERSON_HEIGHT;
                                    //poses_3d[pose_id][kpt_id_where * 4 + 2] = map_3d[2, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HEIGHT
                                    pose_3d[kpt_id_where * 4+2] = features.at<float>(map_3d_id+2, kpt_from_2d_y, kpt_from_2d_x) * AVG_PERSON_HEIGHT;
                                }                                                
                                is_not_break = false;
                            }
                        }
                    }

                    if (is_not_break) {
                        kpt_id_from = limbs[limb].y;

    #ifdef VERBOSE
                        std::cerr << "limb  " << limb << " " << kpt_id_from << std::endl;
                        std::cerr << (poses_2d[pose_id][kpt_id_from * 3 + 2]) << " " << keypoint_treshold << " " << int(poses_2d[pose_id][kpt_id_from * 3 + 2] > keypoint_treshold) << std::endl << std::flush;
    #endif

                        if (poses_2d[pose_id][kpt_id_from * 3 + 2] > keypoint_treshold) {
                        {
                            int kpt_id_where = 0;

                            {
                                kpt_id_where = limbs[limb].x;
                                int map_3d_id = kpt_id_where * 3;
                                int kpt_from_2d_x = int(poses_2d[pose_id][kpt_id_from *3]);
                                int kpt_from_2d_y = int(poses_2d[pose_id][kpt_id_from *3 + 1]);

    #ifdef VERBOSE
                                std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
                                std::cerr << "<< " << kpt_id_from << " " << kpt_id_where << " " << kpt_from_2d_x << "," << kpt_from_2d_y << std::endl << std::flush; 
    #endif

                                //poses_3d[pose_id][kpt_id_where * 4] = map_3d[0, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HEIGHT
                                pose_3d[kpt_id_where * 4] = features.at<float>(map_3d_id, kpt_from_2d_y, kpt_from_2d_x) * AVG_PERSON_HEIGHT;
                                //poses_3d[pose_id][kpt_id_where * 4 + 1] = map_3d[1, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HEIGHT
                                pose_3d[kpt_id_where * 4+1] = features.at<float>(map_3d_id+1, kpt_from_2d_y, kpt_from_2d_x) * AVG_PERSON_HEIGHT;
                                //poses_3d[pose_id][kpt_id_where * 4 + 2] = map_3d[2, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HEIGHT
                                pose_3d[kpt_id_where * 4+2] = features.at<float>(map_3d_id+2, kpt_from_2d_y, kpt_from_2d_x) * AVG_PERSON_HEIGHT;
                            }
                            is_not_break = false;
                        }

                        {
                            int kpt_id_where = 0;                                           
                            {
                                kpt_id_where = limbs[limb].y;
                                int map_3d_id = kpt_id_where * 3;
                                int kpt_from_2d_x = int(poses_2d[pose_id][kpt_id_from *3]);
                                int kpt_from_2d_y = int(poses_2d[pose_id][kpt_id_from *3 + 1]);

    #ifdef VERBOSE
                                std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
                                std::cerr << "<< " << kpt_id_from << " " << kpt_id_where << " " << kpt_from_2d_x << "," << kpt_from_2d_y << std::endl << std::flush; 
    #endif

                                //poses_3d[pose_id][kpt_id_where * 4] = map_3d[0, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HEIGHT
                                pose_3d[kpt_id_where * 4] = features.at<float>(map_3d_id, kpt_from_2d_y, kpt_from_2d_x) * AVG_PERSON_HEIGHT;
                                //poses_3d[pose_id][kpt_id_where * 4 + 1] = map_3d[1, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HEIGHT
                                pose_3d[kpt_id_where * 4+1] = features.at<float>(map_3d_id+1, kpt_from_2d_y, kpt_from_2d_x) * AVG_PERSON_HEIGHT;
                                //poses_3d[pose_id][kpt_id_where * 4 + 2] = map_3d[2, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HEIGHT
                                pose_3d[kpt_id_where * 4+2] = features.at<float>(map_3d_id+2, kpt_from_2d_y, kpt_from_2d_x) * AVG_PERSON_HEIGHT;
                            }
                            is_not_break = false;
                        }

                        {
                            int kpt_id_where = 0;
                            {
                                kpt_id_where = limbs[limb].z;
                                int map_3d_id = kpt_id_where * 3;
                                int kpt_from_2d_x = int(poses_2d[pose_id][kpt_id_from *3]);
                                int kpt_from_2d_y = int(poses_2d[pose_id][kpt_id_from *3 + 1]);

    #ifdef VERBOSE
                                std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
                                std::cerr << "<< " << kpt_id_from << " " << kpt_id_where << " " << kpt_from_2d_x << "," << kpt_from_2d_y << std::endl << std::flush; 
    #endif

                                //poses_3d[pose_id][kpt_id_where * 4] = map_3d[0, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HEIGHT
                                pose_3d[kpt_id_where * 4] = features.at<float>(map_3d_id, kpt_from_2d_y, kpt_from_2d_x) * AVG_PERSON_HEIGHT;
                                //poses_3d[pose_id][kpt_id_where * 4 + 1] = map_3d[1, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HEIGHT
                                pose_3d[kpt_id_where * 4+1] = features.at<float>(map_3d_id+1, kpt_from_2d_y, kpt_from_2d_x) * AVG_PERSON_HEIGHT;
                                //poses_3d[pose_id][kpt_id_where * 4 + 2] = map_3d[2, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HEIGHT
                                pose_3d[kpt_id_where * 4+2] = features.at<float>(map_3d_id+2, kpt_from_2d_y, kpt_from_2d_x) * AVG_PERSON_HEIGHT;
                            }                                                
                            is_not_break = false;
                        }
                        }
                    }

                    if (is_not_break) {
                        kpt_id_from = limbs[limb].z;
    #ifdef VERBOSE                    
                        std::cerr << "limb  " << limb << " " << kpt_id_from << std::endl;
                        std::cerr << (poses_2d[pose_id][kpt_id_from * 3 + 2]) << " " << keypoint_treshold << " " << int(poses_2d[pose_id][kpt_id_from * 3 + 2] > keypoint_treshold) << std::endl << std::flush;
    #endif      
                        if (poses_2d[pose_id][kpt_id_from * 3 + 2] > keypoint_treshold) {
                        {
                            int kpt_id_where = 0;

                            {
                                kpt_id_where = limbs[limb].x;
                                int map_3d_id = kpt_id_where * 3;
                                int kpt_from_2d_x = int(poses_2d[pose_id][kpt_id_from *3]);
                                int kpt_from_2d_y = int(poses_2d[pose_id][kpt_id_from *3 + 1]);


    std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    std::cerr << "<< " << kpt_id_from << " " << kpt_id_where << " " << kpt_from_2d_x << "," << kpt_from_2d_y << std::endl << std::flush; 

                                //poses_3d[pose_id][kpt_id_where * 4] = map_3d[0, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HEIGHT
                                pose_3d[kpt_id_where * 4] = features.at<float>(map_3d_id, kpt_from_2d_y, kpt_from_2d_x) * AVG_PERSON_HEIGHT;
                                //poses_3d[pose_id][kpt_id_where * 4 + 1] = map_3d[1, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HEIGHT
                                pose_3d[kpt_id_where * 4+1] = features.at<float>(map_3d_id+1, kpt_from_2d_y, kpt_from_2d_x) * AVG_PERSON_HEIGHT;
                                //poses_3d[pose_id][kpt_id_where * 4 + 2] = map_3d[2, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HEIGHT
                                pose_3d[kpt_id_where * 4+2] = features.at<float>(map_3d_id+2, kpt_from_2d_y, kpt_from_2d_x) * AVG_PERSON_HEIGHT;
                            }
                            is_not_break = false;
                        }

                        {
                            int kpt_id_where = 0;                    
                            {
                                kpt_id_where = limbs[limb].y;
                                int map_3d_id = kpt_id_where * 3;
                                int kpt_from_2d_x = int(poses_2d[pose_id][kpt_id_from *3]);
                                int kpt_from_2d_y = int(poses_2d[pose_id][kpt_id_from *3 + 1]);


    std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    std::cerr << "<< " << kpt_id_from << " " << kpt_id_where << " " << kpt_from_2d_x << "," << kpt_from_2d_y << std::endl << std::flush; 

                                //poses_3d[pose_id][kpt_id_where * 4] = map_3d[0, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HEIGHT
                                pose_3d[kpt_id_where * 4] = features.at<float>(map_3d_id, kpt_from_2d_y, kpt_from_2d_x) * AVG_PERSON_HEIGHT;
                                //poses_3d[pose_id][kpt_id_where * 4 + 1] = map_3d[1, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HEIGHT
                                pose_3d[kpt_id_where * 4+1] = features.at<float>(map_3d_id+1, kpt_from_2d_y, kpt_from_2d_x) * AVG_PERSON_HEIGHT;
                                //poses_3d[pose_id][kpt_id_where * 4 + 2] = map_3d[2, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HEIGHT
                                pose_3d[kpt_id_where * 4+2] = features.at<float>(map_3d_id+2, kpt_from_2d_y, kpt_from_2d_x) * AVG_PERSON_HEIGHT;
                            }
                            is_not_break = false;
                        }

                        {
                            int kpt_id_where = 0;
                            {
                                kpt_id_where = limbs[limb].z;
                                int map_3d_id = kpt_id_where * 3;
                                int kpt_from_2d_x = int(poses_2d[pose_id][kpt_id_from *3]);
                                int kpt_from_2d_y = int(poses_2d[pose_id][kpt_id_from *3 + 1]);


    std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    std::cerr << "<< " << kpt_id_from << " " << kpt_id_where << " " << kpt_from_2d_x << "," << kpt_from_2d_y << std::endl << std::flush; 

                                //poses_3d[pose_id][kpt_id_where * 4] = map_3d[0, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HEIGHT
                                pose_3d[kpt_id_where * 4] = features.at<float>(map_3d_id, kpt_from_2d_y, kpt_from_2d_x) * AVG_PERSON_HEIGHT;
                                //poses_3d[pose_id][kpt_id_where * 4 + 1] = map_3d[1, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HEIGHT
                                pose_3d[kpt_id_where * 4+1] = features.at<float>(map_3d_id+1, kpt_from_2d_y, kpt_from_2d_x) * AVG_PERSON_HEIGHT;
                                //poses_3d[pose_id][kpt_id_where * 4 + 2] = map_3d[2, kpt_from_2d[1], kpt_from_2d[0]] * AVG_PERSON_HEIGHT
                                pose_3d[kpt_id_where * 4+2] = features.at<float>(map_3d_id+2, kpt_from_2d_y, kpt_from_2d_x) * AVG_PERSON_HEIGHT;
                            }                                                
                            is_not_break = false;
                        }
                    }    

                    }                            
                }

    #ifdef VERBOSE            
                {
                    std::cerr << "3d pose/2 : " << pose_id << std::endl;
                    for (int j=0; j< int(pose_3d.size()); ++j) {
                        std::cerr << pose_3d[j] << ", ";
                    }
                    std::cerr << std::endl << std::flush;
                }
    #endif

                poses_3d.push_back(pose_3d);
            } else {

            }
        }

        root_relative_poses rv;
        rv.poses_2d = poses_2d;
        rv.poses_3d = poses_3d;
        rv.shape[0] = features.size[0];
        rv.shape[1] = features.size[1];
        rv.shape[2] = features.size[2];

#ifdef VERBOSE        
        std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
#endif
        return rv;
    } 

    inline void reshape_transpose(std::vector<std::vector<float>> &dest, const std::vector<float> &src, int n) {
        for (int i=0; i< n; ++i) {
            std::vector<float> line;
            for (int j=0; j*n+i < int(src.size()); ++j) {
                line.push_back(src[j*n+i]);
            } 
            dest.push_back(line);
        }
    }

    parsed_poses parse_poses(
        std::vector<Pose> & previous_poses_2d, PoseCommon & common, 
        const cv::Mat &features, const cv::Mat &heatmap, const cv::Mat &paf_map, float input_scale, int stride, float fx, bool is_video) {

#ifdef VERBOSE            
        std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
#endif

        parsed_poses rv;

        root_relative_poses rrp = get_root_relative_poses(features, heatmap, paf_map);

        auto & poses_2d = rrp.poses_2d;
        auto & poses_3d = rrp.poses_3d;
        auto & shape = rrp.shape;

#ifdef VERBOSE
        std::cerr << "shape:" << shape[0] << "/" << shape[1] << "/" << shape[2] << std::endl << std::flush;
#endif

        std::vector<std::vector<float>> poses_2d_scaled;

#ifdef VERBOSE
        std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
#endif

        for (int i=0; i<int(poses_2d.size()); ++i) {

#ifdef VERBOSE
            std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
#endif

            auto num_kpt = (poses_2d[i].size() - 1) / 3;

#ifdef VERBOSE
            std::cerr << "num_kpt = " << num_kpt << std::endl; 
#endif

            std::vector<float> pose_2d_scaled;
            pose_2d_scaled.resize(poses_2d[i].size());
            for (int j=0; j< int(pose_2d_scaled.size()); ++j) {
                pose_2d_scaled[j] = -1;
            }

            for (int kpt_id = 0; kpt_id < num_kpt; ++kpt_id) {

#ifdef VERBOSE
                std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
#endif
  
                if (poses_2d[i][kpt_id *3 + 2] != -1) {
                            pose_2d_scaled[kpt_id * 3] = int(poses_2d[i][kpt_id * 3] * stride / input_scale);
                            pose_2d_scaled[kpt_id * 3 + 1] = int(poses_2d[i][kpt_id * 3 + 1] * stride / input_scale);
                            pose_2d_scaled[kpt_id * 3 + 2] = poses_2d[i][kpt_id * 3 + 2];
                }
            }

#ifdef VERBOSE
            std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
#endif

            *pose_2d_scaled.rbegin() = *poses_2d[i].rbegin();

#ifdef VERBOSE
            std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
#endif

            poses_2d_scaled.push_back(pose_2d_scaled);

#ifdef VERBOSE
            std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;            
#endif
        }

#ifdef VERBOSE
        std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;

        for (int i=0; i< int(poses_2d_scaled.size()); ++i) {
            for (int j=0; j < int(poses_2d_scaled[i].size()); ++j) {
                std::cerr << poses_2d_scaled[i][j] << ", ";
            }
            std::cerr << std::endl << std::flush;
        }
#endif

        std::vector<Pose> current_poses_2d;
        if (poses_3d.size()) {
            std::cerr << "DIAG: 3D# " << poses_3d.size() << std::endl << std::flush;
        }
        if (is_video) {    
            for (int pose_id = 0; pose_id < int(poses_2d_scaled.size()); ++pose_id) {
                std::vector<std::vector<float>> pose_keypoints;
                for (int i=0; i< common.num_kpts; ++i) {
                    std::vector<float> line { -1,-1};
                    pose_keypoints.push_back(line);
                }
                for (int kpt_id = 0; kpt_id < int(common.num_kpts); ++ kpt_id) {
                    if (poses_2d_scaled[pose_id][kpt_id*3+2] != -1.0) { // keypoint is found
                        pose_keypoints[kpt_id][0] = int(poses_2d_scaled[pose_id][kpt_id * 3 + 0]);
                        pose_keypoints[kpt_id][1] = int(poses_2d_scaled[pose_id][kpt_id * 3 + 1]);
                    }
                }
                Pose pose;
                pose.init(pose_keypoints, *poses_2d_scaled.rbegin());
                current_poses_2d.push_back(pose);
            }
            if (current_poses_2d.size() > 0) {
                std::cerr << "DIAG: 2D# " << current_poses_2d.size() << std::endl << std::flush;
            }
            propagate_ids(common, previous_poses_2d, current_poses_2d);
            if (current_poses_2d.size() > 0) {
                previous_poses_2d = current_poses_2d;
                auto t = CurrentTimeNs();
                std::cerr << "DIAG: dt " << double(t - common.last_poses)*1e-9 << std::endl << std::flush;
                common.last_poses = t;
            } else {
                auto t = CurrentTimeNs();
                if (t - common.last_poses > 10000000000LL) {
                    previous_poses_2d = current_poses_2d;
                    common.last_poses = t;
                }
            }
        }

        std::vector<std::vector<float>> translated_poses_3d;

        for (int pose_id = 0; pose_id < int(poses_3d.size()); ++pose_id) {

#ifdef VERBOSE
            std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
#endif

            std::vector<std::vector<float>> pose_3d;
            reshape_transpose(pose_3d, poses_3d[pose_id], 4);
            std::vector<std::vector<float>> pose_2d;
            reshape_transpose(pose_2d, poses_2d[pose_id], 3);

#ifdef VERBOSE
            std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;

            for (int i=0; i< int(pose_3d.size()); ++i) {
                std::cerr << "pose_3d " << i << " | ";
                for (int j=0; j < int(pose_3d[i].size()); ++j) {
                    std::cerr << pose_3d[i][j] << ", ";
                }
                std::cerr << std::endl << std::flush;
            }
            for (int i=0; i< int(pose_2d.size()); ++i) {
                std::cerr << "pose_2d " << i << " | ";
                for (int j=0; j < int(pose_2d[i].size()); ++j) {
                    std::cerr << pose_2d[i][j] << ", ";
                }
                std::cerr << std::endl << std::flush;
            }

            std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
#endif

        int num_valid = 0;
        for (int i=0; i< int(pose_2d[2].size()); ++i) {
            if (pose_2d[2][i] != -1) {
                ++ num_valid;
            }
        }

#ifdef VERBOSE
        std::cerr << "num valid = " << num_valid << std::endl << std::flush;
#endif

        std::vector<std::vector<float>> pose_3d_valid;
        std::vector<std::vector<float>> pose_2d_valid;

        pose_3d_valid.resize(3);
        for (int i=0; i< 3; ++i) {
            pose_3d_valid[i].resize(num_valid);
            for (int j=0; j< num_valid; ++j) {
                pose_3d_valid[i][j] = 0;
            }
        }

        pose_2d_valid.resize(2);
        for (int i=0; i< 2; ++i) {
            pose_2d_valid[i].resize(num_valid);
            for (int j=0; j< num_valid; ++j) {
                pose_2d_valid[i][j] = 0;
            }
        }

        int valid_id = 0;

        // for kpt_id in range(pose_3d.shape[1]):
        for (int kpt_id = 0; kpt_id < (pose_3d[0].size()); ++ kpt_id) {
            if (pose_2d[2][kpt_id] == -1)
                continue;
            for (int i=0; i<3; ++i)
                pose_3d_valid[i][valid_id] = pose_3d[i][kpt_id];
            for (int i=0; i<2; ++i)
                pose_2d_valid[i][valid_id] = pose_2d[i][kpt_id];
            valid_id += 1;
        }

#ifdef VERBOSE
        for (int i=0; i< int(pose_3d_valid.size()); ++i) {
            std::cerr << "pose_3d_valid " << i << " | ";
            for (int j=0; j < int(pose_3d[i].size()); ++j) {
                std::cerr << pose_3d[i][j] << ", ";
            }
            std::cerr << std::endl << std::flush;
        }

        for (int i=0; i< int(pose_2d_valid.size()); ++i) {
            std::cerr << "pose_2d_valid " << i << " | ";
            for (int j=0; j < int(pose_2d_valid[i].size()); ++j) {
                std::cerr << pose_2d_valid[i][j] << ", ";
            }
            std::cerr << std::endl << std::flush;
        }
#endif

        for (int i=0; i< int(pose_2d_valid[0].size()); ++i)
            pose_2d_valid[0][i] = pose_2d_valid[0][i] - shape[2]*0.5;
        for (int i=0; i< int(pose_2d_valid[1].size()); ++i)
            pose_2d_valid[1][i] = pose_2d_valid[1][i] - shape[1]*0.5;

        std::vector<float> mean_3d;
        std::vector<float> mean_2d;

        mean_3d.resize(3);
        for (int i=0; i<3; ++i) {
            float acc = 0.0;
            for (int j=0; j< int(pose_3d_valid[i].size()); ++j) {
                acc += pose_3d_valid[i][j];
            }
            mean_3d[i] = acc / float(pose_3d_valid[i].size());
        }

        mean_2d.resize(2);
        for (int i=0; i<2; ++i) {
            float acc = 0.0;
            for (int j=0; j< int(pose_2d_valid[i].size()); ++j) {
                acc += pose_2d_valid[i][j];
            }
            mean_2d[i] = acc / float(pose_2d_valid[i].size());
        }

#ifdef VERBOSE
        std::cerr << "mean_3d: " << mean_3d[0] << "," << mean_3d[1] << "," << mean_3d[2] << std::endl << std::flush;
        std::cerr << "mean_2d: " << mean_2d[0] << "," << mean_2d[1] << std::endl << std::flush;
        std::cerr << "num valid = " << num_valid << std::endl << std::flush;

        for (int i=0; i< int(pose_3d_valid.size()); ++i) {
            std::cerr << "pose_3d_valid " << i << " | ";
            for (int j=0; j < int(pose_3d[i].size()); ++j) {
                std::cerr << pose_3d[i][j] << ", ";
            }
            std::cerr << std::endl << std::flush;
        }

        for (int i=0; i< int(pose_2d_valid.size()); ++i) {
            std::cerr << "pose_2d_valid " << i << " | ";
            for (int j=0; j < int(pose_2d_valid[i].size()); ++j) {
                std::cerr << pose_2d_valid[i][j] << ", ";
            }
            std::cerr << std::endl << std::flush;
        }
#endif

        float numerator = 0.0;
        for(int i=0; i<2; ++i) {
            for (int j=0; j<(pose_3d_valid[i].size()); ++j) {
                float elt = pose_3d_valid[i][j] - mean_3d[i];
                numerator += elt * elt;
            }
        }


        numerator = std::sqrt(numerator);
        float denominator = 0.0;
        for(int i=0; i<2; ++i) {
            for (int j=0; j<(pose_2d_valid[i].size()); ++j) {
                float elt = pose_2d_valid[i][j] - mean_2d[i];
                denominator += elt * elt;
            }
        }

        denominator = std::sqrt(denominator);

#ifdef VERBOSE
        std::cerr << "numerator:" << numerator << std::endl << std::flush; 
        std::cerr << "denominator:" << denominator << std::endl << std::flush; 
        std::cerr << "fx = " << fx << " input_scale = " << input_scale << " stride = " << stride << std::endl << std::flush;
#endif

        std::vector<float> mean_2d_ { mean_2d[0], mean_2d[1], fx*input_scale / stride};
        std::vector<float> mean_3d_ { mean_3d[0], mean_3d[1], 0};
        std::vector<float> translation{ numerator / denominator * mean_2d_[0] - mean_3d_[0], numerator / denominator * mean_2d_[1] - mean_3d_[1] , numerator / denominator * mean_2d_[2] - mean_3d_[2]};

#ifdef VERBOSE
        std::cerr << "translation:" << translation[0] << "," << translation[1] << "," << translation[2] << std::endl << std::flush; // ok
#endif

        if (is_video) {
            translation = current_poses_2d[pose_id].filter(translation);
        }

        for (int kpt_id =0; kpt_id <19 ; ++ kpt_id ) {
            pose_3d[0][kpt_id ] += translation[0];
            pose_3d[1][kpt_id ] += translation[1];
            pose_3d[2][kpt_id ] += translation[2];
        }

        std::vector<float> flatp3d;
        for (int i=0; i< int(pose_3d[0].size()); ++i) {
            for (int j=0; j<4; ++j) {
                flatp3d.push_back(pose_3d[j][i]);
            }
        }

#ifdef VERBOSE
        std::cerr << "flatp3d " << std::endl;
        for (int i=0; i< int(flatp3d.size()); ++i)
            std::cerr << flatp3d[i] << ", ";
        std::cerr << std::endl << std::flush;
#endif

        translated_poses_3d.push_back(flatp3d);
        if (pose_id >= 0 && pose_id < int(current_poses_2d.size())) {
            rv.ids.push_back(current_poses_2d[pose_id].id);
        } else {
            rv.ids.push_back(-1);
        }
    }

    rv.poses_2d_scaled = poses_2d_scaled;
    rv.translated_poses_3d = translated_poses_3d;

#ifdef VERBOSE
    std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
#endif
    return rv;
}

} // namespace
} // moetsi::ssp