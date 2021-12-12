// Copyright (C) 2018-2019 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
//

#pragma once

#include <vector>
#include <iostream>
#include <opencv2/core/core.hpp>

namespace moetsi::ssp {
namespace human_pose_estimation {

constexpr double pi = 3.14159265358979323846;

inline float get_alpha(float rate = 30.0, float cutoff = 1) {
  float tau = 1.0 / (2.0 * pi * cutoff);
  float te = 1.0 / rate;
  return 1.0 / (1.0 + tau / te);
}

struct LowPassFilter {
  bool has_previous = false;
  float previous = 0;

  inline float operator ()(float x, float alpha = 0.5) {
    if (!has_previous) {
      has_previous = true;
      previous = x;
      return x;
    }
    float x_filtered = alpha * x + (1.0 - alpha) * previous;
    previous = x_filtered;
    return x_filtered;
  }
};

struct OneEuroFilter {
  float freq;
  float mincutoff;
  float beta;
  float dcutoff;
  LowPassFilter filter_x;
  LowPassFilter filter_dx;
  bool has_previous;
  float previous;
  float dx;

  void init(float freq_ = 15, float mincutoff_ = 1, float beta_ = 1, float dcutoff_=1) {
    freq = freq_;
    mincutoff = mincutoff_;
    beta = beta_;
    dcutoff = dcutoff_;
    has_previous = false;
    previous = 0;
    dx = 0;
  }

  float operator()(float x) {
    if (!has_previous) {
      dx = 0;
      has_previous = true;
    } else {
      dx = (x - previous) * freq;
    }
    float dx_smoothed = filter_dx(dx, get_alpha(freq, dcutoff));
    float cutoff = mincutoff + beta * abs(dx_smoothed);
    float x_filtered = filter_x(x, get_alpha(freq, cutoff));
    previous = x;
    return x_filtered;
  }
};

struct PoseCommon {
    int num_kpts = 18;
    std::vector<std::string> kpt_names {"neck", "nose",
                 "l_sho", "l_elb", "l_wri", "l_hip", "l_knee", "l_ank",
                 "r_sho", "r_elb", "r_wri", "r_hip", "r_knee", "r_ank",
                 "r_eye", "l_eye",
                 "r_ear", "l_ear"};
    std::vector<float> sigmas { 0.079, 0.026, 0.079, 0.072, 0.062, 0.107, 0.087, 0.089, 0.079, 0.072, 0.062, 0.107, 0.087, 0.089, 0.025, 0.025, 0.035, 0.035 };
    std::vector<float> vars;
    int last_id = -1;
    int color[3] {0, 224, 255};

    PoseCommon() {
      for (auto &s: sigmas) {
        vars.push_back((s*2.0)*(s*2.0));
      }
    }
};

template<typename T>
inline std::vector<int> int_vector(const std::vector<T> & v) {
  std::vector<int> v2;
  v2.resize(v.size());
  for (int i=0; i< int(v.size()); ++i) {
    v2[i] = int(v[i]);
  }
  return v2;
}

struct Pose {
    std::vector<std::vector<float>> keypoints;
    std::vector<float> confidence;

    std::vector<std::vector<float>> bbox;
    std::vector<OneEuroFilter> translation_filter;
    int id = -1; // -1 is None for us...
   
    void init(const std::vector<std::vector<float>> &keypoints_, const std::vector<float> &confidence_) {

      keypoints = keypoints_;
      confidence = confidence_;

      std::vector<std::vector<int>> found_keypoints;
      found_keypoints.resize(2);
      int nkeypoints = 0;
      for (int n=0; n<int(keypoints.size()); ++n) {
        if (keypoints[n][0] != -1) {
          ++nkeypoints;
        }
      }
      found_keypoints.resize(nkeypoints);
      int found_kpt_id = 0;
      for (int kpt_id=0; kpt_id < int(keypoints.size()); ++kpt_id) {
        if (keypoints[kpt_id][0] == -1) {
          continue;
        }
        found_keypoints[found_kpt_id] = int_vector(keypoints[kpt_id]);
        ++ found_kpt_id;
      }
      // make a bounding rect
      std::vector<float> maxes;
      maxes.resize(keypoints[0].size());
      for (int i=0; i< int(maxes.size()); ++i) {
        maxes[i] = -1e12;
      }
      std::vector<float> mines;
      mines.resize(keypoints[0].size());
      for (int i=0; i< int(mines.size()); ++i) {
        mines[i] = 1e12;
      }

      for (int i=0; i< int(keypoints.size()); ++i) {
        for (int j =0; j < int(keypoints[0].size()); ++j) {
          if (keypoints[i][j] > maxes[j]) {
            maxes[j] = keypoints[i][j];
          }
          if (keypoints[i][j] < mines[j]) {
            mines[j] = keypoints[i][j];                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
          }
        }
      }

      bbox = std::vector<std::vector<float>> { maxes, mines };
      id = -1;
      translation_filter.resize(3);
      translation_filter[0].init(80, 1.0, 0.01);
      translation_filter[1].init(80, 1.0, 0.01);
      translation_filter[2].init(80, 1.0, 0.01);
    }

    void update_id(PoseCommon &pc, int id_ = -1) {
      id = id_;
      if (id == -1) {
        id = pc.last_id + 1;
        pc.last_id += 1;
      }
    }

    std::vector<float> filter(const std::vector<float> &f) {
      std::vector<float> rv;
      for (int i=0; i< int(translation_filter.size()); ++i) {
        rv.push_back(translation_filter[i](f[i]));
      }
      return rv;
    }
};

constexpr float eps = 1e-7; // np.spacing(1)

inline int get_similarity(const PoseCommon & common, const Pose &a, const Pose &b, float threashold=0.5) {
    int num_similar_kpt = 0;
    for (int kpt_id = 0; kpt_id < common.num_kpts; ++kpt_id) {
        float distance = 0.0;
        for (int i=0; i< int(a.keypoints[0].size()); ++i) {
            float dx = a.keypoints[kpt_id][i] - b.keypoints[kpt_id][i];
            float dx2 = dx*dx;
            distance += dx2;
        }
        float area = (a.bbox[1][0] - a.bbox[0][0])*(a.bbox[1][1] - a.bbox[0][1])*(a.bbox[1][2] - a.bbox[0][2]);
        float similarity = std::exp( - distance / (2.0 * (area + eps) * common.vars[kpt_id]));
        if (similarity > threashold) {
            ++ num_similar_kpt;
        }
    }

    return num_similar_kpt;
}

inline void propagate_ids(PoseCommon & common, std::vector<Pose> & previous_poses, std::vector<Pose> &current_poses, int threshold=3) {
    /* Propagate poses ids from previous frame results. Id is propagated,
    if there are at least `threshold` similar keypoints between pose from previous frame and current.

    :param previous_poses: poses from previous frame with ids
    :param current_poses: poses from current frame to assign ids
    :param threshold: minimal number of similar keypoints between poses
    :return: None
    */

    std::vector<int> current_poses_sorted_ids;
    for (int i=0; i< int(current_poses.size()); ++i) {
        current_poses_sorted_ids.push_back(i);
    }
    std::sort(current_poses_sorted_ids.begin(), current_poses_sorted_ids.end(), [&](int a, int b) {
        return current_poses[a].confidence > current_poses[b].confidence;
    });

    //if (current_poses.size() > 0) {
    //    assert(current_poses[0].confidence >= current_poses[1].confidence);
    //}

    std::vector<uint8_t> mask;
    for (int i=0; i< int(previous_poses.size()); ++i) {
        mask.push_back(1);
    }

    for (int current_pose_id_id = 0; current_pose_id_id < int(current_poses_sorted_ids.size()); ++current_pose_id_id){
        int current_pose_id = current_poses_sorted_ids[current_pose_id_id];
        int best_matched_id = -1; // None
        int best_matched_pose_id = -1; // None
        float best_matched_iou = 0;

        for (int previous_pose_id = 0; previous_pose_id < int(previous_poses.size()); ++ previous_pose_id) {
            if (mask[previous_pose_id] == 0)
                continue;
            float iou = get_similarity(common, current_poses[current_pose_id], previous_poses[previous_pose_id]);
            if (iou > best_matched_iou) {
                best_matched_iou = iou;
                best_matched_pose_id = previous_poses[previous_pose_id].id;
                best_matched_id = previous_pose_id;
            }
        }
        if (best_matched_iou >= threshold) {
            mask[best_matched_id] = 0;
        } else {
            best_matched_pose_id = -1;
        }
        current_poses[current_pose_id].update_id(common, best_matched_pose_id);
        if (best_matched_pose_id > -1) { //is not None:
            current_poses[current_pose_id].translation_filter = previous_poses[best_matched_id].translation_filter;
        }
    }
}

struct HumanPose {
    HumanPose(const std::vector<cv::Point3f>& keypoints = std::vector<cv::Point3f>(),
              const float& score = 0);

    std::vector<cv::Point3f> keypoints;
    float score;
};
} // namespace human_pose_estimation

#include <vector>
#include <opencv2/core/core.hpp>

namespace human_pose_estimation {
std::vector<HumanPose> extractPoses(
        std::vector<cv::Mat>& heatMaps,
        std::vector<cv::Mat>& pafs,
        int upsampleRatio);
} // namespace human_pose_estimation
// Copyright (C) 2018-2019 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
//

#include <vector>
#include <opencv2/core/core.hpp>

namespace human_pose_estimation {
struct Peak {
    Peak(const int id = -1,
         const cv::Point2f& pos = cv::Point2f(),
         const float score = 0.0f);

    int id;
    cv::Point2f pos;
    float score;
};

struct HumanPoseByPeaksIndices {
    explicit HumanPoseByPeaksIndices(const int keypointsNumber);

    std::vector<int> peaksIndices;
    int nJoints;
    float score;
};

struct TwoJointsConnection {
    TwoJointsConnection(const int firstJointIdx,
                        const int secondJointIdx,
                        const float score);

    int firstJointIdx;
    int secondJointIdx;
    float score;
};

void findPeaks(const std::vector<cv::Mat>& heatMaps,
               const float minPeaksDistance,
               std::vector<std::vector<Peak> >& allPeaks,
               int heatMapId);

std::vector<HumanPose> groupPeaksToPoses(
        const std::vector<std::vector<Peak> >& allPeaks,
        const std::vector<cv::Mat>& pafs,
        const size_t keypointsNumber,
        const float midPointsScoreThreshold,
        const float foundMidPointsRatioThreshold,
        const int minJointsNumber,
        const float minSubsetScore);
} // namespace human_pose_estimation

#include <opencv2/core/core.hpp>

namespace human_pose_estimation {
    struct root_relative_poses {

        std::vector<std::vector<float>> poses_2d;
        std::vector<std::vector<float>> poses_3d;

        int shape[3];
    };

    // features, heatmap, paf_map = inference_results
    root_relative_poses get_root_relative_poses(const cv::Mat &features, const cv::Mat &heatmap, const cv::Mat &paf_map);

    struct parsed_poses {
         std::vector<std::vector<float>>  translated_poses_3d;
         std::vector<std::vector<float>>  poses_2d_scaled;
    };

    parsed_poses parse_poses(std::vector<Pose> & previous_poses_2d, PoseCommon & common, 
        const cv::Mat &features, const cv::Mat &heatmap, const cv::Mat &paf_map, float input_scale, int stride, float fx, bool is_video=false);
}

/* here we implement the following
        if fx < 0:  # Focal length is unknown
            fx = np.float32(0.8 * frame.shape[1])

        inference_result = net.infer(scaled_img)
        poses_3d, poses_2d = parse_poses(inference_result, input_scale, stride, fx, is_video)

            poses_3d = rotate_poses(poses_3d, R, t)
            poses_3d_copy = poses_3d.copy()
            x = poses_3d_copy[:, 0::4]
            y = poses_3d_copy[:, 1::4]
            z = poses_3d_copy[:, 2::4]
            poses_3d[:, 0::4], poses_3d[:, 1::4], poses_3d[:, 2::4] = -z, x, -y

            poses_3d = poses_3d.reshape(poses_3d.shape[0], 19, -1)[:, :, 0:3] //THIS IS WHAT THE FUNCTION EXPECTS
*/

namespace human_pose_estimation {
    struct poses {
         std::vector<std::vector<float>>  poses_3d;
         std::vector<std::vector<float>>  poses_2d;      
    };

    struct vector3 {
      float x,y,z;
    };

    struct matrix3x4 {
      float mat[3][4]; // index is: line, column

      inline void mul(vector3 & dest, const vector3 & src) const {
        dest.x = src.x * mat[0][0]
                + src.y * mat[0][1]
                + src.z * mat[0][2]
                + mat[0][3];

        dest.y = src.x * mat[1][0]
                + src.y * mat[1][1]
                + src.z * mat[1][2]
                + mat[1][3];

        dest.z = src.x * mat[2][0]
                + src.y * mat[2][1]
                + src.z * mat[2][2]
                + mat[2][3];                

      }
    };

    inline poses parse_poses(std::vector<Pose> & previous_poses_2d, PoseCommon & common, // these two on the left are a state 
        const matrix3x4 &R, 
        const cv::Mat &features, const cv::Mat &heatmap, const cv::Mat &paf_map, float input_scale, int stride, 
        float fx, float frame_shape_1,
        bool is_video=false) {

        float nfx = fx;
        if (nfx < 0) nfx = 0.8 * frame_shape_1;
        auto p = parse_poses(previous_poses_2d, common, features, heatmap, paf_map, input_scale, stride, nfx, is_video);

        poses rv;
        rv.poses_2d = p.poses_2d_scaled;

        int c = 0;
        for (auto &l: p.translated_poses_3d) {

std::cerr << " final[ ";
for (auto &ll: l) {
  std::cerr << ll << " ";
}
std::cerr << " ]" << std::endl << std::flush;

for (int i=0; i<l.size(); i+=4) {

  auto x = l[i];
  auto y = l[i+1];
  auto z = l[i+2];
std::vector<float> l2 { float(-z*0.01), float(x*0.01), float(-y*0.01) };
rv.poses_3d.push_back(l2);

}
      /*    vector3 v;
          v.x = l[0];
          v.y = l[1];
          v.z = l[2];

          std::cerr << "before R " << l[0] << " " << l[1] << " " << l[2] << std::endl << std::flush; 
          vector3 vr;
          R.mul(vr, v);
          // -z, x, -y
          std::vector<float> l2 { -vr.z, vr.x, -vr.y };
          rv.poses_3d.push_back(l2); */

          if (++c > 0) break;
        }

        return rv;
    }
}

} // moetsi::ssp