// Copyright (C) 2018-2023 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
//

#include "../include/tracker.hpp"

#include <stdlib.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <ostream>
#include <set>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "../utils/include/utils/kuhn_munkres.hpp"
#include "../utils/include/utils/ocv_common.hpp"

#include "../include/core.hpp"
#include "../include/descriptor.hpp"
#include "../include/distance.hpp"
#include "../include/utils.hpp"

namespace {
cv::Point Center(const cv::Rect& rect) {
    return cv::Point(static_cast<int>(rect.x + rect.width * 0.5), static_cast<int>(rect.y + rect.height * 0.5));
}

std::vector<cv::Point> Centers(const TrackedObjects& detections) {
    std::vector<cv::Point> centers(detections.size());
    for (size_t i = 0; i < detections.size(); i++) {
        centers[i] = Center(detections[i].rect);
    }
    return centers;
}

DetectionLog ConvertTracksToDetectionLog(const ObjectTracks& tracks) {
    DetectionLog log;

    // Combine detected objects by respective frame indices.
    std::map<int64_t, TrackedObjects> objects;
    for (const auto& track : tracks)
        for (const auto& object : track.second) {
            auto itr = objects.find(object.frame_idx);
            if (itr != objects.end())
                itr->second.emplace_back(object);
            else
                objects.emplace(object.frame_idx, TrackedObjects{object});
        }

    for (const auto& frame_res : objects) {
        DetectionLogEntry entry;
        entry.frame_idx = frame_res.first;
        entry.objects = std::move(frame_res.second);
        log.push_back(std::move(entry));
    }

    return log;
}

inline bool IsInRange(float val, float min, float max) {
    return min <= val && val <= max;
}

inline bool IsInRange(float val, cv::Vec2f range) {
    return IsInRange(val, range[0], range[1]);
}

std::vector<cv::Scalar> GenRandomColors(int colors_num) {
    std::vector<cv::Scalar> colors(colors_num);
    for (int i = 0; i < colors_num; i++) {
        colors[i] = cv::Scalar(static_cast<uchar>(255. * rand() / RAND_MAX),  // NOLINT
                               static_cast<uchar>(255. * rand() / RAND_MAX),  // NOLINT
                               static_cast<uchar>(255. * rand() / RAND_MAX));  // NOLINT
    }
    return colors;
}

}  // anonymous namespace

TrackerParams::TrackerParams()
    : min_track_duration(0),
      forget_delay(150),
      aff_thr_fast(0.8f),
    //   aff_thr_strong(0.75f),
      aff_thr_strong(0.0f),
      shape_affinity_w(0.5f),
      motion_affinity_w(0.2f),
      time_affinity_w(0.0f),
      min_det_conf(0.62f),
      bbox_aspect_ratios_range(0.666f, 5.0f),
      bbox_heights_range(40, 1000),
      predict(25),
    //   strong_affinity_thr(0.2805f),
      strong_affinity_thr(0.0f),
      reid_thr(0.61f),
      drop_forgotten_tracks(true),
      max_num_objects_in_track(300) {}

void ValidateParams(const TrackerParams& p) {
    PT_CHECK_GE(p.min_track_duration, static_cast<size_t>(0));
    PT_CHECK_LE(p.min_track_duration, static_cast<size_t>(10000));

    PT_CHECK_LE(p.forget_delay, static_cast<size_t>(10000));

    PT_CHECK_GE(p.aff_thr_fast, 0.0f);
    PT_CHECK_LE(p.aff_thr_fast, 1.0f);

    PT_CHECK_GE(p.aff_thr_strong, 0.0f);
    PT_CHECK_LE(p.aff_thr_strong, 1.0f);

    PT_CHECK_GE(p.shape_affinity_w, 0.0f);
    PT_CHECK_LE(p.shape_affinity_w, 100.0f);

    PT_CHECK_GE(p.motion_affinity_w, 0.0f);
    PT_CHECK_LE(p.motion_affinity_w, 100.0f);

    PT_CHECK_GE(p.time_affinity_w, 0.0f);
    PT_CHECK_LE(p.time_affinity_w, 100.0f);

    PT_CHECK_GE(p.min_det_conf, 0.0f);
    PT_CHECK_LE(p.min_det_conf, 1.0f);

    PT_CHECK_GE(p.bbox_aspect_ratios_range[0], 0.0f);
    PT_CHECK_LE(p.bbox_aspect_ratios_range[1], 10.0f);
    PT_CHECK_LT(p.bbox_aspect_ratios_range[0], p.bbox_aspect_ratios_range[1]);

    PT_CHECK_GE(p.bbox_heights_range[0], 10.0f);
    PT_CHECK_LE(p.bbox_heights_range[1], 1080.0f);
    PT_CHECK_LT(p.bbox_heights_range[0], p.bbox_heights_range[1]);

    PT_CHECK_GE(p.predict, 0);
    PT_CHECK_LE(p.predict, 10000);

    PT_CHECK_GE(p.strong_affinity_thr, 0.0f);
    PT_CHECK_LE(p.strong_affinity_thr, 1.0f);

    PT_CHECK_GE(p.reid_thr, 0.0f);
    PT_CHECK_LE(p.reid_thr, 1.0f);

    if (p.max_num_objects_in_track > 0) {
        int min_required_track_length = static_cast<int>(p.forget_delay);
        PT_CHECK_GE(p.max_num_objects_in_track, min_required_track_length);
        PT_CHECK_LE(p.max_num_objects_in_track, 10000);
    }
}

PedestrianTracker::PedestrianTracker(const TrackerParams& params, const cv::Size& frame_size)
    : params_(params),
      descriptor_strong_(nullptr),
      distance_strong_(nullptr),
      tracks_counter_(0),
      frame_size_(frame_size),
      prev_frame_size_(frame_size),
      prev_timestamp_(std::numeric_limits<uint64_t>::max()) {
    ValidateParams(params);
}

// Pipeline parameters getter.
const TrackerParams& PedestrianTracker::params() const {
    return params_;
}

// Descriptor fast getter.
const PedestrianTracker::Descriptor& PedestrianTracker::descriptor_fast() const {
    return descriptor_fast_;
}

// Descriptor fast setter.
void PedestrianTracker::set_descriptor_fast(const Descriptor& val) {
    descriptor_fast_ = val;
}

// Descriptor strong getter.
const PedestrianTracker::Descriptor& PedestrianTracker::descriptor_strong() const {
    return descriptor_strong_;
}

// Descriptor strong setter.
void PedestrianTracker::set_descriptor_strong(const Descriptor& val) {
    descriptor_strong_ = val;
}

// Distance fast getter.
const PedestrianTracker::Distance& PedestrianTracker::distance_fast() const {
    return distance_fast_;
}

// Distance fast setter.
void PedestrianTracker::set_distance_fast(const Distance& val) {
    distance_fast_ = val;
}

// Distance strong getter.
const PedestrianTracker::Distance& PedestrianTracker::distance_strong() const {
    return distance_strong_;
}

// Distance strong setter.
void PedestrianTracker::set_distance_strong(const Distance& val) {
    distance_strong_ = val;
}

// Returns all tracks including forgotten (lost too many frames ago).
const std::unordered_map<size_t, Track>& PedestrianTracker::tracks() const {
    return tracks_;
}

// Returns indexes of active tracks only.
const std::set<size_t>& PedestrianTracker::active_track_ids() const {
    return active_track_ids_;
}

/**
 * This function retrieves the most recent detections for tracks identified by detected_track_ids_.
 * It constructs a vector of detection_struct_t, each representing the latest detection in a track.
 * 
 * The detected_track_ids_ is a set that contains the IDs of tracks that have been detected in the current frame.
 * It is cleared at the beginning of each frame processing in the Process method to ensure it only contains
 * tracks detected in the current frame. During the execution of the Process method, each detection is analyzed to determine
 * if it corresponds to an existing track or if it necessitates the creation of a new track. If a detection is successfully matched
 * to an existing track, the ID of that track is added to detected_track_ids_. Similarly, if a detection does not match any existing
 * track and leads to the creation of a new track, the ID of this newly created track is also added to detected_track_ids_.
 * The matching process involves comparing the detection with track descriptors using a set of criteria, including spatial proximity
 * and appearance similarity, to ensure accurate tracking. This mechanism ensures that detected_track_ids_ is dynamically updated to
 * reflect the most current state of track detections within the frame being processed.
 * This way, detected_track_ids_ always reflects the most recent state of detected tracks within the current frame.
 * 
 * @return A vector of moetsi::ssp::detection_struct_t, each representing the most recent detection of a tracked object.
 */
std::vector<moetsi::ssp::detection_struct_t> PedestrianTracker::GetMostRecentDetections() const {
    // Initialize a vector to hold the most recent detections.
    std::vector<moetsi::ssp::detection_struct_t> most_recent_detections;
    
    // Iterate over each track ID stored in detected_track_ids_.
    for (auto track_id : detected_track_ids_) {
        // Check if the current track ID exists in the tracks_ map.
        if (tracks_.find(track_id) != tracks_.end()) {
            // Retrieve the last detection (most recent) for the current track.
            auto last_det = tracks_.at(track_id).objects.back();
            
            // Create a detection_struct_t object to represent the most recent detection.
            moetsi::ssp::detection_struct_t det;
            det.device_time = last_det.timestamp; // Set the timestamp of the detection to the last detection's timestamp.
            det.sensor_track_id = track_id; // Set the track ID.
            det.sensor_center_x = last_det.center_x; // Set the center X coordinate of the detection.
            det.sensor_center_y = last_det.center_y; // Set the center Y coordinate of the detection.
            det.sensor_center_z = last_det.center_z; // Set the center Z coordinate of the detection.
            det.confidence = last_det.confidence; // Set the confidence of the detection.
            det.detection_label = 1; // Set the detection label. Currently hardcoded as 1.

            // Set the strong descriptor using a pointer for more efficient copying
            std::memcpy(det.strong_descriptor, last_det.strong_descriptor.ptr<float>(), 256 * sizeof(float));

            // Set the rect values
            det.rect_x = last_det.rect.x;
            det.rect_y = last_det.rect.y;
            det.rect_width = last_det.rect.width;
            det.rect_height = last_det.rect.height;
            
            // Add the constructed detection to the vector of most recent detections.
            most_recent_detections.push_back(det);
        }
    }
    // Return the vector containing the most recent detections for the tracked objects.
    return most_recent_detections;
}

// Returns detection log which is used for tracks saving.
DetectionLog PedestrianTracker::GetDetectionLog(const bool valid_only) const {
    return ConvertTracksToDetectionLog(all_tracks(valid_only));
}

/**
 * Filters detections based on confidence, aspect ratio, and height.
 * This function iterates through all provided detections and applies a series of filters to each one.
 * A detection is only added to the list of filtered detections if it passes all the following criteria:
 * 1. Its confidence score is above a predefined minimum.
 * 2. Its aspect ratio (height divided by width) falls within a specified range.
 * 3. Its height falls within a specified range.
 * If a detection fails any of these tests, it is excluded from the filtered list, and a message is printed indicating the reason for its exclusion.
 * 
 * @param detections A vector of detections to be filtered.
 * @return A vector of detections that have passed all filters.
 */
TrackedObjects PedestrianTracker::FilterDetections(const TrackedObjects& detections) const {
    TrackedObjects filtered_detections;
    for (const auto& det : detections) {
        // Calculate the aspect ratio of the detection.
        float aspect_ratio = static_cast<float>(det.rect.height) / det.rect.width;
        
        // Check if the detection passes all filters.
        if (det.confidence > params_.min_det_conf) {
            if (IsInRange(aspect_ratio, params_.bbox_aspect_ratios_range)) {
                if (IsInRange(static_cast<float>(det.rect.height), params_.bbox_heights_range)) {
                    // If the detection passes all filters, add it to the list of filtered detections.
                    filtered_detections.emplace_back(det);
                } else {
                    // Print a message if the detection's height is out of the specified range, including expected and actual values.
                    // std::cerr << "Detection filtered out due to height not in range. Expected: " << params_.bbox_heights_range[0] << " to " << params_.bbox_heights_range[1] << ", Actual: " << det.rect.height << std::endl;
                }
            } else {
                // Print a message if the detection's aspect ratio is out of the specified range, including expected and actual values.
                // std::cerr << "Detection filtered out due to aspect ratio not in range. Expected: " << params_.bbox_aspect_ratios_range[0] << " to " << params_.bbox_aspect_ratios_range[1] << ", Actual: " << aspect_ratio << std::endl;
            }
        } else {
            // Print a message if the detection's confidence is below the minimum, including expected and actual values.
            // std::cerr << "Detection filtered out due to low confidence. Expected: >" << params_.min_det_conf << ", Actual: " << det.confidence << std::endl;
        }
    }
    return filtered_detections;
}

/**
 * Solves the assignment problem between existing tracks and new detections.
 * This function aims to find the best match between existing tracks and new detections based on the dissimilarity matrix.
 * It updates the sets of unmatched tracks, unmatched detections, and matches between tracks and detections.
 * 
 * @param track_ids Set of existing track IDs. These are the IDs of tracks that have been previously identified and are currently being tracked through the scene.
 * @param detections Vector of detected objects in the current frame. These objects represent potential updates to existing tracks or new objects that have entered the scene.
 * @param descriptors Vector of descriptors for the detected objects. Descriptors are used to quantify and compare the characteristics of detected objects for matching purposes.
 * @param thr Threshold for matching (not used in this function but could be used for filtering matches). Instead of using this threshold, the method relies on a dissimilarity matrix and the Kuhn-Munkres algorithm to determine the best matches based on a calculated score, where a lower score indicates a better match.
 * @param unmatched_tracks Output parameter to hold the set of track IDs that couldn't be matched with any detection. These tracks remain unmatched when no current detection closely resembles them, as determined by the dissimilarity scores exceeding a predefined threshold, which might suggest that the tracked object has left the field of view or is temporarily obscured.
 * @param unmatched_detections Output parameter to hold the set of detection indices that couldn't be matched with any track. These are new detections in the scene that do not closely match any existing tracks based on the dissimilarity scores, indicating potential new objects that have not been tracked previously.
 * @param matches Output parameter to hold the set of matches found. Each match is a tuple containing track ID, detection index, and match score. The match score is derived from the dissimilarity matrix, with the Kuhn-Munkres algorithm determining the optimal pairing between tracks and detections. A match is made when the dissimilarity score is sufficiently low, indicating a high degree of similarity between a track and a detection.
 */
void PedestrianTracker::SolveAssignmentProblem(const std::set<size_t>& track_ids,
                                               const TrackedObjects& detections,
                                               const std::vector<cv::Mat>& descriptors,
                                               float thr,
                                               std::set<size_t>* unmatched_tracks,
                                               std::set<size_t>* unmatched_detections,
                                               std::set<std::tuple<size_t, size_t, float>>* matches) {
    // Ensure the pointers to output parameters are not null.
    PT_CHECK(unmatched_tracks);
    PT_CHECK(unmatched_detections);
    PT_CHECK(matches);

    // Clear the output parameters to ensure they are empty before starting the assignment.
    unmatched_tracks->clear();
    unmatched_detections->clear();
    matches->clear();

    // Check the preconditions for the assignment problem.
    PT_CHECK(!track_ids.empty());
    PT_CHECK(!detections.empty());
    PT_CHECK(descriptors.size() == detections.size());

    // First, we compute the dissimilarity matrix between tracks and detections to understand how similar or different they are.
    // The ComputeDissimilarityMatrix function is tasked with this calculation. It iterates over each pair of track and detection,
    // computing a score that quantifies their dissimilarity. The function fills a matrix where each cell [i, j] contains the dissimilarity score
    // between the i-th track and the j-th detection. In this matrix, a lower score signifies a higher similarity, suggesting a potential match.
    // For example, if the score at [2, 3] is 0.1, it means the third detection is very similar to the second track.
    cv::Mat dissimilarity;
    ComputeDissimilarityMatrix(track_ids, detections, descriptors, &dissimilarity);


    // The KuhnMunkres().Solve Rturns a vector of indices, where each index corresponds to a track.
    // The value at each index is the index of the matched detection r a special value (such as -1) for no assignment
    // For instance, if the result vector is [2, 0, -1, 3], it means:
    // - The first track is best matched with the third detection.
    // - The second track is best matched with the first detection.
    // - The third track has no suitable match (assuming -1 indicates no match).
    // - The fourth track is best matched with the fourth detection.
    std::vector<size_t> res = KuhnMunkres().Solve(dissimilarity);

    // Mark all detections as initially unmatched by inserting their indices into the unmatched_detections set.
    for (size_t i = 0; i < detections.size(); i++) {
        unmatched_detections->insert(i);
    }
    // Iterate over each track ID to process the assignment results from the Kuhn-Munkres algorithm.
    size_t i = 0;
    for (size_t id : track_ids) {
        // If the result index is within the range of detections, it means a match has been found.
        if (res[i] < detections.size()) {
            // Calculate the match score as 1 minus the dissimilarity value.
            // A match is found when the algorithm assigns a detection to a track with a dissimilarity score that is not the special "no match" value.
            // The match score is calculated as 1 minus the dissimilarity to convert it into a similarity score (higher is better).
            float match_score = 1 - dissimilarity.at<float>(i, res[i]);
            // Record the match by adding it to the matches set, including the track ID, detection index, and match score.
            matches->emplace(id, res[i], match_score);
            // Remove the matched detection from the set of unmatched detections, as it has now been assigned to a track.
            unmatched_detections->erase(res[i]);
        } else {
            // If the result index is not within the range of detections, mark the track as unmatched.
            // A track is considered unmatched if the algorithm assigns the special "no match" value to it, indicating no suitable detection was found.
            unmatched_tracks->insert(id);
        }
        i++;
    }
    //We print the size of matches we have
    // std::cerr << "We have " << matches->size() << " matches" << std::endl;
    // Now we print the amount of unmatched detections and the amount of unmatched tracks
    // std::cerr << "We have " << unmatched_detections->size() << " unmatched DETECTIONS and " << unmatched_tracks->size() << " unmatched TRACKS" << std::endl;
}

const ObjectTracks PedestrianTracker::all_tracks(bool valid_only) const {
    ObjectTracks all_objects;
    size_t counter = 0;

    std::set<size_t> sorted_ids;
    for (const auto& pair : tracks()) {
        sorted_ids.emplace(pair.first);
    }

    for (size_t id : sorted_ids) {
        if (!valid_only || IsTrackValid(id)) {
            TrackedObjects filtered_objects;
            for (const auto& object : tracks().at(id).objects) {
                filtered_objects.emplace_back(object);
                filtered_objects.back().object_id = counter;
            }
            all_objects.emplace(counter++, filtered_objects);
        }
    }
    return all_objects;
}

cv::Rect PedestrianTracker::PredictRect(size_t id, size_t k, size_t s) const {
    const auto& track = tracks_.at(id);
    PT_CHECK(!track.empty());

    if (track.size() == 1) {
        return track[0].rect;
    }

    size_t start_i = track.size() > k ? track.size() - k : 0;
    float width = 0, height = 0;

    for (size_t i = start_i; i < track.size(); i++) {
        width += track[i].rect.width;
        height += track[i].rect.height;
    }

    PT_CHECK(track.size() - start_i > 0);
    width /= (track.size() - start_i);
    height /= (track.size() - start_i);

    float delim = 0;
    cv::Point2f d(0, 0);

    for (size_t i = start_i + 1; i < track.size(); i++) {
        d += cv::Point2f(Center(track[i].rect) - Center(track[i - 1].rect));
        delim += (track[i].frame_idx - track[i - 1].frame_idx);
    }

    if (delim) {
        d /= delim;
    }

    s += 1;

    cv::Point c = Center(track.back().rect);
    return cv::Rect(static_cast<int>(c.x - width / 2 + d.x * s),
                    static_cast<int>(c.y - height / 2 + d.y * s),
                    static_cast<int>(width),
                    static_cast<int>(height));
}

/**
 * @brief Erases a track if its bounding box is out of the frame boundaries.
 * 
 * This function checks if the predicted bounding box of a track, identified by track_id,
 * is outside the boundaries of the previous frame. If it is, the track is marked as lost
 * by setting its 'lost' status to a value greater than the forget delay, effectively
 * scheduling it for deletion. It also cleans up any distance measurements related to this
 * track from the global distance map and removes the track ID from the list of active track IDs.
 * CURRENTLY THIS MEANS THAT TRACKS THAT MOVE OUT OF FRAME ARE NOT SEARCHED AGAINST AGAIN
 * 
 * @param track_id The unique identifier of the track to potentially erase.
 * @return true if the track was erased because its bounding box is out of frame, false otherwise.
 */
bool PedestrianTracker::EraseTrackIfBBoxIsOutOfFrame(size_t track_id) {
    // Check if the track exists in the tracks_ map.
    if (tracks_.find(track_id) == tracks_.end())
        return true; // Return true indicating the track is considered erased/non-existent.
    
    // Calculate the center of the predicted bounding box for the track.
    auto c = Center(tracks_.at(track_id).predicted_rect);
    
    // Check if the center of the bounding box is outside the previous frame's boundaries.
    if (!isSizeEmpty(prev_frame_size_) &&
        (c.x < 0 || c.y < 0 || c.x > prev_frame_size_.width || c.y > prev_frame_size_.height)) {
        // Mark the track as lost by setting its 'lost' status beyond the forget delay.
        tracks_.at(track_id).lost = params_.forget_delay + 1; // Global variable 'tracks_' is modified here.
        
        // Erase distance measurements related to this track from the global distance map.
        for (auto id : active_track_ids()) {
            size_t min_id = std::min(id, track_id);
            size_t max_id = std::max(id, track_id);
            tracks_dists_.erase(std::pair<size_t, size_t>(min_id, max_id)); // Global variable 'tracks_dists_' is modified here.
        }
        
        // Remove the track ID from the list of active track IDs.
        active_track_ids_.erase(track_id); // Global variable 'active_track_ids_' is modified here.
        
        return true; // Return true indicating the track was erased.
    }
    return false; // Return false if the track's bounding box is within frame boundaries.
}

bool PedestrianTracker::EraseTrackIfItWasLostTooManyFramesAgo(size_t track_id) {
    if (tracks_.find(track_id) == tracks_.end())
        return true;
    if (tracks_.at(track_id).lost > params_.forget_delay) {
        for (auto id : active_track_ids()) {
            size_t min_id = std::min(id, track_id);
            size_t max_id = std::max(id, track_id);
            tracks_dists_.erase(std::pair<size_t, size_t>(min_id, max_id));
        }
        active_track_ids_.erase(track_id);

        return true;
    }
    return false;
}

bool PedestrianTracker::UpdateLostTrackAndEraseIfItsNeeded(size_t track_id) {
    tracks_.at(track_id).lost++;
    tracks_.at(track_id).predicted_rect = PredictRect(track_id, params().predict, tracks_.at(track_id).lost);

    bool erased = EraseTrackIfBBoxIsOutOfFrame(track_id);
    if (!erased)
        erased = EraseTrackIfItWasLostTooManyFramesAgo(track_id);
    return erased;
}

void PedestrianTracker::UpdateLostTracks(const std::set<size_t>& track_ids) {
    for (auto track_id : track_ids) {
        UpdateLostTrackAndEraseIfItsNeeded(track_id);
    }
}

void PedestrianTracker::ComputeFastDescriptorsUsingAttachedDescriptors(const TrackedObjects& detections,
                                                                       std::vector<cv::Mat>* descriptors_fast) {
    PT_CHECK(descriptors_fast);
    descriptors_fast->clear();
    descriptors_fast->reserve(detections.size());
    for (const auto& det : detections) {
        descriptors_fast->emplace_back(det.fast_descriptor);
    }
}

void PedestrianTracker::ComputeStrongDescriptorsUsingAttachedDescriptors(const TrackedObjects& detections,
                                                 std::vector<cv::Mat>* descriptors_strong) {

    PT_CHECK(descriptors_strong);
    descriptors_strong->clear();
    descriptors_strong->reserve(detections.size());
    for (const auto& det : detections) {
        descriptors_strong->emplace_back(det.strong_descriptor);
    }
}

void PedestrianTracker::Process(const TrackedObjects& input_detections, uint64_t timestamp) {
    // Clearing detected track IDs for the current processing cycle
    detected_track_ids_.clear();

    // Checking if this is not the first frame being processed
    if (prev_timestamp_ != std::numeric_limits<uint64_t>::max())
    {
        // Ensuring the current timestamp is greater than the previous to maintain temporal order
        // PT_CHECK_LT(prev_timestamp_, timestamp);
    }

    // Filtering detections based on predefined criteria. The filters applied include:
    // 1. Confidence threshold: Only detections with a confidence score higher than params_.min_det_conf are kept.
    // 2. Aspect ratio filter: Detections are filtered based on their aspect ratio, which must fall within the range defined by params_.bbox_aspect_ratios_range.
    // 3. Height filter: The height of the detection's bounding box must fall within the range defined by params_.bbox_heights_range.
    TrackedObjects detections = FilterDetections(input_detections);

    // Preparing fast descriptors for the detections
    std::vector<cv::Mat> descriptors_fast;
    // This method extracts fast descriptors directly from the detections without additional computation
    ComputeFastDescriptorsUsingAttachedDescriptors(detections, &descriptors_fast);

    // The active_track_ids_ set is crucial for tracking the IDs of currently active tracks within the PedestrianTracker class.
    // EraseTrackIfBBoxIsOutOfFrame when called removes a track from active_track_ids_ if bounding box is out of frame
    // EraseTrackIfItWasLostTooManyFramesAgo removes a track from active_track_ids_ if it was lost too many frames ago
    // DropForgottenTracks drops tracks that have been forgotten (i.e., lost for too long) and optionally reassigns IDs to active tracks.
    // AddNewTrack adds a track to active_track_ids
    auto active_tracks = active_track_ids_;

    // If there are active tracks and detections, attempt to match them
    if (!active_tracks.empty() && !detections.empty()) {
        // Sets to keep track of unmatched tracks and detections
        std::set<size_t> unmatched_tracks, unmatched_detections;
        // Matches between tracks and detections along with their confidence scores
        std::set<std::tuple<size_t, size_t, float>> matches;

        // Solving the assignment problem to find matches between active tracks and new detections
        SolveAssignmentProblem(active_tracks,
                               detections,
                               descriptors_fast,
                               params_.aff_thr_fast,
                               &unmatched_tracks,
                               &unmatched_detections,
                               &matches);

        // Map to keep track of whether a track has a strong match with a detection
        std::map<size_t, std::pair<bool, cv::Mat>> is_matching_to_track;

        // If strong matching is enabled, perform additional matching based on strong descriptors
        if (distance_strong_) {
            // Get the track to detection matches
            std::vector<std::pair<size_t, size_t>> grey_area_track_to_det_matches = GetTrackToDetectionIds(matches);
            if (grey_area_track_to_det_matches.size() > 0) {
                is_matching_to_track = StrongMatching(detections, grey_area_track_to_det_matches);                
            }
        }

        // Processing matches to update tracks or create new ones
        for (const auto& match : matches) {
            size_t track_id = std::get<0>(match);
            size_t det_id = std::get<1>(match);
            float conf = std::get<2>(match);

            // Updating the last detected object in the track with the predicted position
            auto last_det = tracks_.at(track_id).objects.back();
            last_det.rect = tracks_.at(track_id).predicted_rect;

            // If the match confidence is high enough, append the detection to the track
            if (conf > params_.aff_thr_fast) {
                AppendToTrack(track_id, detections[det_id], detections[det_id].fast_descriptor, detections[det_id].strong_descriptor);
                unmatched_detections.erase(det_id);
            } else {
                // For lower confidence, check if strong matching is applicable
                if (conf > params_.strong_affinity_thr) {
                    if (distance_strong_ && is_matching_to_track[track_id].first) {
                        // If there's a strong match, append the detection with its strong descriptor
                        AppendToTrack(track_id,
                                      detections[det_id],
                                      detections[det_id].fast_descriptor,
                                      detections[det_id].strong_descriptor);
                    } else {
                        // If no strong match, consider the track lost and start a new track
                        if (UpdateLostTrackAndEraseIfItsNeeded(track_id)) {
                            AddNewTrack(detections[det_id],
                                        detections[det_id].fast_descriptor,
                                        detections[det_id].strong_descriptor);
                        }
                        else {
                            AddNewTrack(detections[det_id],
                                        detections[det_id].fast_descriptor,
                                        detections[det_id].strong_descriptor);
                        }
                    }

                    unmatched_detections.erase(det_id);
                } else {
                    // If the confidence is too low, mark the track as unmatched
                    unmatched_tracks.insert(track_id);
                    // The track <> detection that did not make the cutoff is now made to a new track
                    AddNewTrack(detections[det_id],
                                detections[det_id].fast_descriptor,
                                detections[det_id].strong_descriptor);
                }
            }
        }

        // Attempt to create new tracks for unmatched detections
        AddNewTracks(detections, descriptors_fast, unmatched_detections);
        // Update the status of tracks that were not matched to any detection (+1 for track.lost value)
        UpdateLostTracks(unmatched_tracks);

        // Check if any active tracks have bounding boxes out of frame and erase them if necessary
        for (size_t id : active_tracks) {
            //TODO: See if this negatively effects anything. It should only allow us to track in and out of frame
            // EraseTrackIfBBoxIsOutOfFrame(id);
        }
    } else {
        // If there are no active tracks and/or detections, attempt to create new tracks for all detections
        AddNewTracks(detections, descriptors_fast);
        // Update the status of all tracks that didn't have detections (+1 for track.lost value)
        UpdateLostTracks(active_tracks);
    }

    // Update the size of the previous frame for future checks
    // prev_frame_size_ = frame.size();
    // Optionally drop tracks that have been lost for too long
    if (params_.drop_forgotten_tracks)
        DropForgottenTracks();

    // Clear distances between tracks for the next processing cycle
    tracks_dists_.clear();
    // Update the timestamp of the previous frame
    prev_timestamp_ = timestamp;
}

/**
 * Drops tracks that have been forgotten (i.e., lost for too long) and optionally reassigns IDs to active tracks.
 * This function iterates through all tracks, removing those that are deemed forgotten based on a predefined criterion.
 * If the maximum track ID exceeds a certain threshold, it reassigns IDs to all remaining tracks starting from 0 to keep the IDs compact.
 */
void PedestrianTracker::DropForgottenTracks() {
    std::unordered_map<size_t, Track> new_tracks; // Temporary storage for tracks that are not forgotten
    std::set<size_t> new_active_tracks; // Temporary storage for IDs of active tracks

    size_t max_id = 0; // Holds the maximum track ID encountered
    // Find the maximum track ID if there are any active tracks
    if (!active_track_ids_.empty())
        max_id = *std::max_element(active_track_ids_.begin(), active_track_ids_.end());

    const size_t kMaxTrackID = 10000; // Threshold for maximum track ID
    bool reassign_id = max_id > kMaxTrackID; // Determine if ID reassignment is necessary

    size_t counter = 0; // Counter for reassigning track IDs
    // Iterate through all tracks to filter out forgotten ones and potentially reassign IDs
    for (const auto& pair : tracks_) {
        // Check if the track is not forgotten
        if (!IsTrackForgotten(pair.first)) {
            // Add the track to new_tracks with either a new ID or its original ID
            new_tracks.emplace(reassign_id ? counter : pair.first, pair.second);
            // Similarly, add the track ID to new_active_tracks
            new_active_tracks.emplace(reassign_id ? counter : pair.first);
            counter++; // Increment counter for the next potential ID reassignment
        }
    }
    // Replace the current tracks and active track IDs with the filtered and potentially reassigned ones
    tracks_.swap(new_tracks);
    active_track_ids_.swap(new_active_tracks);

    // Update the global track counter based on whether IDs were reassigned
    tracks_counter_ = reassign_id ? counter : tracks_counter_;
}

float PedestrianTracker::ShapeAffinity(float weight, const cv::Rect& trk, const cv::Rect& det) {
    float w_dist = static_cast<float>(std::abs(trk.width - det.width) / (trk.width + det.width));
    float h_dist = static_cast<float>(std::abs(trk.height - det.height) / (trk.height + det.height));
    return static_cast<float>(exp(static_cast<double>(-weight * (w_dist + h_dist))));
}

float PedestrianTracker::MotionAffinity(float weight, const cv::Rect& trk, const cv::Rect& det) {
    float x_dist = static_cast<float>(trk.x - det.x) * (trk.x - det.x) / (det.width * det.width);
    float y_dist = static_cast<float>(trk.y - det.y) * (trk.y - det.y) / (det.height * det.height);
    return static_cast<float>(exp(static_cast<double>(-weight * (x_dist + y_dist))));
}

float PedestrianTracker::TimeAffinity(float weight, const float& trk_time, const float& det_time) {
    return static_cast<float>(exp(static_cast<double>(-weight * std::fabs(trk_time - det_time))));
}

void PedestrianTracker::ComputeFastDesciptors(const cv::Mat& frame,
                                              const TrackedObjects& detections,
                                              std::vector<cv::Mat>* descriptors) {
    *descriptors = std::vector<cv::Mat>(detections.size(), cv::Mat());
    for (size_t i = 0; i < detections.size(); i++) {
        descriptor_fast_->Compute(frame(detections[i].rect).clone(), &((*descriptors)[i]));
    }
}

void PedestrianTracker::ComputeDissimilarityMatrix(const std::set<size_t>& active_tracks,
                                                   const TrackedObjects& detections,
                                                   const std::vector<cv::Mat>& descriptors_fast,
                                                   cv::Mat* dissimilarity_matrix) {
    cv::Mat am(active_tracks.size(), detections.size(), CV_32F, cv::Scalar(0));
    size_t i = 0;
    for (auto id : active_tracks) {
        auto ptr = am.ptr<float>(i);
        for (size_t j = 0; j < descriptors_fast.size(); j++) {
            auto last_det = tracks_.at(id).objects.back();
            last_det.rect = tracks_.at(id).predicted_rect;
            ptr[j] = AffinityFast(tracks_.at(id).descriptor_fast, last_det, descriptors_fast[j], detections[j]);
        }
        i++;
    }
    *dissimilarity_matrix = 1.0 - am;
}

std::vector<float> PedestrianTracker::ComputeDistances(const TrackedObjects& detections,
                                                       const std::vector<std::pair<size_t, size_t>>& track_and_det_ids) {

    // Vectors to store the descriptors to compute distances between.
    std::vector<cv::Mat> descriptors1;
    std::vector<cv::Mat> descriptors2;

    // Loop through all pairs of track and detection IDs again.
    for (size_t i = 0; i < track_and_det_ids.size(); i++) {
        // Extract track and detection IDs from each pair.
        size_t track_id = track_and_det_ids[i].first;
        size_t det_id = track_and_det_ids[i].second;

        // If the strong descriptor of the current track is empty...
        if (tracks_.at(track_id).descriptor_strong.empty()) {
            // Assign the corresponding descriptor from the descriptors vector to the track's descriptor_strong.
            std::cerr << "!!!!!Descriptor Strong is empty and it should never be" << std::endl;
            tracks_.at(track_id).descriptor_strong = detections[det_id].strong_descriptor.clone();
        }

        // Push the current detection's descriptor to the descriptors1 vector.
        descriptors1.push_back(detections[det_id].strong_descriptor);
        // Push the current track's strong descriptor to the descriptors2 vector.
        descriptors2.push_back(tracks_.at(track_id).descriptor_strong);

    }


    // Compute the distances between corresponding descriptors in descriptors1 and descriptors2.
    std::vector<float> distances = distance_strong_->Compute(descriptors1, descriptors2);

    //Now we have to print the distances
    for (size_t i = 0; i < distances.size(); i++) {
        std::cerr << "Distance " << i << ": " << distances[i] << std::endl;
    }

    // Return the computed distances.
    return distances;
}

// std::vector<float> PedestrianTracker::ComputeDistances(const TrackedObjects& detections,
//                                                        const std::vector<std::pair<size_t, size_t>>& track_and_det_ids) {
//     // Map objects to store batch IDs for each detection and track.
//     std::map<size_t, size_t> det_to_batch_ids;
//     std::map<size_t, size_t> track_to_batch_ids;

//     // std::vector<cv::Mat> images;

//     // Vector to store descriptors of each detection and track.
//     std::vector<cv::Mat> descriptors;

//     // Loop through all pairs of track and detection IDs.
//     for (size_t i = 0; i < track_and_det_ids.size(); i++) {
//         // Extract track and detection IDs from each pair.
//         size_t track_id = track_and_det_ids[i].first;
//         size_t det_id = track_and_det_ids[i].second;

//         // If the strong descriptor of the current track is empty...
//         if (tracks_.at(track_id).descriptor_strong.empty()) {

//             // images.push_back(tracks_.at(track_id).last_image);
//             std::cerr << "THE DESCRIPTION STRONG IS EMPTY (should never happen because when we add track we should set it)" << std::endl;
//             // Add a new empty cv::Mat to the descriptors vector.
//             descriptors.push_back(cv::Mat());
//             // Store the current index of the descriptors vector as the batch ID for the current track.
//             track_to_batch_ids[track_id] = descriptors.size() - 1;
//         }

//         // Regardless of the condition above, add a new empty cv::Mat to the descriptors vector.
//         descriptors.push_back(cv::Mat());
//         // Store the current index of the descriptors vector as the batch ID for the current detection.
//         det_to_batch_ids[det_id] = descriptors.size() - 1;
//     }

//     // Previously this was the step where the cropped detection images had their feature descriptions processed with ML
//     // descriptor_strong_->Compute(images, &descriptors);
//     // Populate the descriptors vector with actual strong descriptors from the TrackedObjects.
//     ComputeStrongDescriptorsUsingAttachedDescriptors(detections, &descriptors);

//     // Vectors to store the descriptors to compute distances between.
//     std::vector<cv::Mat> descriptors1;
//     std::vector<cv::Mat> descriptors2;

//     // Loop through all pairs of track and detection IDs again.
//     for (size_t i = 0; i < track_and_det_ids.size(); i++) {
//         // Extract track and detection IDs from each pair.
//         size_t track_id = track_and_det_ids[i].first;
//         size_t det_id = track_and_det_ids[i].second;

//         // If the strong descriptor of the current track is empty...
//         if (tracks_.at(track_id).descriptor_strong.empty()) {
//             // Assign the corresponding descriptor from the descriptors vector to the track's descriptor_strong.
//             std::cerr << "Descriptor Strong is empty and it should never be" << std::endl;
//             tracks_.at(track_id).descriptor_strong = descriptors[track_to_batch_ids[track_id]].clone();
//         }

//         // Push the current detection's descriptor to the descriptors1 vector.
//         descriptors1.push_back(descriptors[det_to_batch_ids[det_id]]);
//         // Push the current track's strong descriptor to the descriptors2 vector.
//         descriptors2.push_back(tracks_.at(track_id).descriptor_strong);
//     }


//     // Compute the distances between corresponding descriptors in descriptors1 and descriptors2.
//     std::vector<float> distances = distance_strong_->Compute(descriptors1, descriptors2);

//     // Return the computed distances.
//     return distances;
// }

/**
 * Extracts pairs of track and detection IDs based on matching confidence thresholds for further processing.
 * 
 * This function filters matches between tracks and detections using two key thresholds:
 * - params_.aff_thr_fast: the affinity threshold for fast matching, typically used for quick, less accurate matches.
 * - params_.strong_affinity_thr: the threshold for strong affinity matching, used for more accurate, discriminative matching.
 * 
 * The filtering criteria are as follows:
 * - Matches with a confidence score below params_.aff_thr_fast are considered too weak for reliable fast matching. These matches are not immediately trusted and are subjected to further verification.
 * - Matches with a confidence score above params_.strong_affinity_thr but below params_.aff_thr_fast are in a 'gray zone'. They are not strong enough to be accepted outright without further verification, yet not so weak as to be discarded. These matches are considered for strong matching to verify their validity.
 * - Matches with a confidence score below params_.strong_affinity_thr are considered too weak even for strong matching. These are likely false positives or matches of very low quality and are thus excluded from the vector returned by this function.
 * 
 * The purpose of this function is to identify matches that are uncertain and might benefit from additional verification through strong matching. By applying these thresholds, the algorithm aims to balance between discarding weak matches and verifying potential matches that are not immediately clear.
 * 
 * @param matches A set of tuples containing track IDs, detection IDs, and their matching confidence scores.
 * @return A vector of pairs, each consisting of a track ID and a detection ID, filtered based on the specified thresholds. Which is the "grey" area which is below fast threshold but above strong treshold. Below strong gets dropped as a false positive.
 * 
 * The resulting vector is used downstream in the Process method to perform strong matching on these uncertain matches,
 * potentially correcting or confirming the initial match based on more discriminative features. This step is crucial for improving the accuracy of the tracking system by ensuring that only matches with a high degree of confidence are accepted.
 */
std::vector<std::pair<size_t, size_t>> PedestrianTracker::GetTrackToDetectionIds(
    const std::set<std::tuple<size_t, size_t, float>>& matches) {
    std::vector<std::pair<size_t, size_t>> track_and_det_ids;

    for (const auto& match : matches) {
        size_t track_id = std::get<0>(match);
        size_t det_id = std::get<1>(match);
        float conf = std::get<2>(match);
        // Now we print whether this match was above the fast threshold, below the fast threshold but above the strong threshold, or below the strong threshold
        if (conf > params_.aff_thr_fast)
        {
            // std::cerr << "a match is above the fast threshold" << std::endl;
        }
        // Include matches where the confidence score is below the fast matching threshold but above the strong affinity threshold.
        // This filters out matches that are too weak for any reliable matching, focusing on those that are in the 'gray zone'.
        else if (conf < params_.aff_thr_fast && conf > params_.strong_affinity_thr) {
            track_and_det_ids.emplace_back(track_id, det_id);
            // std::cerr << "a match is below the fast threshold but above the strong threshold" << std::endl;
        }
        else if (conf < params_.strong_affinity_thr) {
            // TODO: This doesn't mean we can't still try the strong_affinity because it is already calculated. We can just reduce the strong_affinity_thr to 0
            // and then we can try to match the detections to the tracks.
            // std::cerr << "a match is below the strong threshold" << std::endl;
        }
    }
    return track_and_det_ids;
}

/**
 * Performs strong matching between tracks and detections using re-identification features.
 * 
 * This function evaluates potential matches between tracks and detections based on re-identification (ReID) features.
 * It computes the ReID distances between each pair of track and detection, then assesses whether the pair matches
 * based on the ReID distance and an affinity score. The function returns a map where each entry corresponds to a track ID
 * and contains a pair indicating whether a strong match was found and the descriptor of the matched detection.
 * 
 * The map `is_matching` plays a crucial role in this process. For each track, it stores whether a strong match was found
 * and the descriptor of the detection that was matched. This information is used downstream to update the track with the new detection
 * or to make decisions about track creation or deletion.
 * 
 * @param detections A list of detected objects in the current frame.
 * @param track_and_det_ids A list of pairs of track IDs and detection IDs that potentially match. Result of filtering potential matches by GetTrackToDetectionIds.
 * @return A map from track IDs to pairs indicating whether a strong match was found and the descriptor of the matched detection.
 */
std::map<size_t, std::pair<bool, cv::Mat>> PedestrianTracker::StrongMatching(
    const TrackedObjects& detections,
    const std::vector<std::pair<size_t, size_t>>& track_and_det_ids) {
    // Initialize a map to store the matching result for each track. The boolean indicates if a strong match was found.
    // If the boolean is true, the cv::Mat holds the descriptor of the detection that was strongly matched to the track, used for further processing or track updating.
    // If the boolean is false, the cv::Mat may be empty, indicating no strong match was found or the detection did not have a descriptor that met the strong matching criteria.
    // An empty cv::Mat in the case of a false boolean typically occurs when the detection is considered too weak or irrelevant for the current tracking context.
    std::map<size_t, std::pair<bool, cv::Mat>> is_matching;

    // If there are no potential matches, return the empty map immediately. This check prevents unnecessary computation.
    // Shouldn't be triggered because we do a check before we call this but we do it in case.
    if (track_and_det_ids.size() == 0) {
        return is_matching;                       
    }

    // Compute descriptors for detections. This map associates detection IDs with their descriptors, which are used
    // descriptors of the detections. These distances are used to assess the strength of the match between each track and detection.
    std::vector<float> distances = ComputeDistances(detections, track_and_det_ids);

    // Iterate over each potential match to determine if it's a strong match based on computed distances and affinity scores.
    for (size_t i = 0; i < track_and_det_ids.size(); i++) {
        // Calculate the affinity based on ReID distance. A lower distance indicates a higher similarity between the track and detection.
        auto reid_affinity = 1.0 - distances[i];

        // Extract the track ID and detection ID from the current potential match.
        size_t track_id = track_and_det_ids[i].first;
        size_t det_id = track_and_det_ids[i].second;

        // Retrieve the track and detection objects using their IDs to compute spatial affinity.
        const auto& track = tracks_.at(track_id);
        const auto& detection = detections[det_id];

        // Get the last detection in the track and update its bounding box to the predicted position for affinity calculation.
        auto last_det = track.objects.back();
        last_det.rect = track.predicted_rect;

        // Compute the overall affinity score by combining ReID affinity with spatial affinity. This score determines the strength of the match.
        float affinity = static_cast<float>(reid_affinity) * Affinity(last_det, detection);
        std::cerr << "Affinity: " << Affinity(last_det, detection) << std::endl;

        // Determine if the detection matches the track based on affinity thresholds. A match is considered strong if it surpasses both thresholds.
        bool is_detection_matching = reid_affinity > params_.reid_thr && affinity > params_.aff_thr_strong;
        // We print reid_thr and reid_affinity
        std::cerr << "reid_thr: " << params_.reid_thr << ", reid_affinity: " << reid_affinity << ", aff_thr_strong: " << params_.aff_thr_strong << ", affinity: " << affinity << std::endl;

        // Store the matching result and the detection's descriptor in the map. If a strong match is found, the detection's descriptor
        // is used to update the track or for further processing.
        is_matching[track_id] = std::pair<bool, cv::Mat>(is_detection_matching, detection.strong_descriptor);
    }
    // Return the map containing the matching results for each track. This map is used downstream to update tracks with new detections or make decisions about track management.
    return is_matching;
}

/**
 * Adds new tracks for all detections.
 * This method is typically called when all detections in a frame are considered new tracks.
 * This scenario occurs in the initial frames of a video or when no existing tracks are found to match with the current detections.
 * 
 * @param detections A vector containing all detected objects in the frame.
 * @param descriptors_fast A vector containing the fast descriptors for each detection.
 */
void PedestrianTracker::AddNewTracks(const TrackedObjects& detections,
                                     const std::vector<cv::Mat>& descriptors_fast) {
    PT_CHECK(detections.size() == descriptors_fast.size());
    for (size_t i = 0; i < detections.size(); i++) {
        AddNewTrack(detections[i], descriptors_fast[i]);
    }
}

/**
 * Adds new tracks for a subset of detections.
 * This method is called when only specific detections are considered for new tracks, typically after matching existing tracks with detections.
 * The unmatched detections are then passed to this method to initiate new tracks.
 * This is used in scenarios where the Process method has identified detections that do not match any existing tracks, indicating they are new objects entering the scene.
 * 
 * @param detections A vector containing all detected objects in the frame.
 * @param descriptors_fast A vector containing the fast descriptors for each detection.
 * @param ids A set containing the indices of detections that are to be added as new tracks.
 */
void PedestrianTracker::AddNewTracks(const TrackedObjects& detections,
                                     const std::vector<cv::Mat>& descriptors_fast,
                                     const std::set<size_t>& ids) {
    PT_CHECK(detections.size() == descriptors_fast.size());
    for (size_t i : ids) {
        PT_CHECK(i < detections.size());
        AddNewTrack(detections[i], descriptors_fast[i]);
    }
}

// void PedestrianTracker::AddNewTrack(const cv::Mat& frame,
//                                     const TrackedObject& detection,
//                                     const cv::Mat& descriptor_fast,
//                                     const cv::Mat& descriptor_strong) {
//     auto detection_with_id = detection;
//     detection_with_id.object_id = tracks_counter_;
//     tracks_.emplace(std::pair<size_t, Track>(
//         tracks_counter_,
//         Track({detection_with_id}, frame(detection.rect).clone(), descriptor_fast.clone(), descriptor_strong.clone())));

//    // After Track is created and emplaced, checking whether descriptor_strong is empty or not
//     if (descriptor_strong.empty()) {
//         std::cerr << "new track - descriptor_strong is EMPTY" << std::endl;
//     } else {
//         std::cerr << "new track - descriptor_strong is NOT EMPTY" << std::endl;
//     }


//     for (size_t id : active_track_ids_) {
//         tracks_dists_.emplace(std::pair<size_t, size_t>(id, tracks_counter_), std::numeric_limits<float>::max());
//     }

//     active_track_ids_.insert(tracks_counter_);
//     tracks_counter_++;
// }

void PedestrianTracker::AddNewTrack(const TrackedObject& detection,
                                    const cv::Mat& descriptor_fast,
                                    const cv::Mat& descriptor_strong) {
    // Create a copy of the detection and set the object_id
    auto detection_with_id = detection;
    detection_with_id.object_id = tracks_counter_;
    
    // Create a new Track
    Track newTrack({detection_with_id}, 
                   descriptor_fast.clone(), 
                   detection.strong_descriptor.clone());
    
    // Add the new Track to the tracks_ map
    tracks_.emplace(std::pair<size_t, Track>(tracks_counter_, std::move(newTrack)));
    
    // Update the distances and active track IDs
    for (size_t id : active_track_ids_) {
        tracks_dists_.emplace(std::pair<size_t, size_t>(id, tracks_counter_), std::numeric_limits<float>::max());
    }
    
    active_track_ids_.insert(tracks_counter_);

    // We add the id to the detected track ids
    detected_track_ids_.push_back(tracks_counter_);

    // Increment the tracks counter
    tracks_counter_++;
}

void PedestrianTracker::AppendToTrack(size_t track_id,
                                      const TrackedObject& detection,
                                      const cv::Mat& descriptor_fast,
                                      const cv::Mat& descriptor_strong) {
    // Ensure the track is not already forgotten (i.e., removed after being lost for too long)
    PT_CHECK(!IsTrackForgotten(track_id));

    // Add the current track ID to the list of detected track IDs for this frame.
    // This is crucial for the Process method to keep track of which IDs have been updated in the current frame.
    detected_track_ids_.push_back(track_id);

    // Copy the detection object and assign the current track ID to it.
    // This step is necessary to maintain a consistent ID across all detections within a track.
    auto detection_with_id = detection;
    detection_with_id.object_id = track_id;

    // Retrieve the current track from the tracks_ map using the track ID.
    auto& cur_track = tracks_.at(track_id);
    // Add the updated detection to the track's list of objects.
    cur_track.objects.emplace_back(detection_with_id);
    // Update the track's predicted rectangle to the current detection's rectangle.
    cur_track.predicted_rect = detection.rect;
    // Reset the 'lost' counter since the track has been updated with a new detection.
    cur_track.lost = 0;

    // Update the last image of the track with the current frame's relevant section.
    // FYI: commented out saving last image because it required the frame and doesn't seem to do anything
    // cur_track.last_image = frame(detection.rect).clone();
    // Update the fast descriptor with the current detection's descriptor.
    cur_track.descriptor_fast = descriptor_fast.clone();
    // Increment the length of the track to reflect the addition of a new detection.
    cur_track.length++;

    // If the strong descriptor of the track is empty, initialize it with the current detection's strong descriptor.
    // Otherwise, if the current detection's strong descriptor is not empty, update the track's strong descriptor
    // by averaging it with the current detection's strong descriptor.
    if (cur_track.descriptor_strong.empty()) {
        cur_track.descriptor_strong = descriptor_strong.clone();
    } else if (!descriptor_strong.empty()) {
        // TODO: See if there is a better way to update descriptor_strong
        cur_track.descriptor_strong = 0.5 * (descriptor_strong + cur_track.descriptor_strong);
    }

    // If there's a limit on the number of objects a track can contain, enforce this limit.
    // This is done by removing the oldest detections until the track contains at most the maximum allowed number of objects.
    if (params_.max_num_objects_in_track > 0) {
        while (cur_track.objects.size() > static_cast<size_t>(params_.max_num_objects_in_track)) {
            cur_track.objects.erase(cur_track.objects.begin());
        }
    }
}

float PedestrianTracker::AffinityFast(const cv::Mat& descriptor1,
                                      const TrackedObject& obj1,
                                      const cv::Mat& descriptor2,
                                      const TrackedObject& obj2) {
    const float eps = 1e-6f;
    float shp_aff = ShapeAffinity(params_.shape_affinity_w, obj1.rect, obj2.rect);
    if (shp_aff < eps)
        return 0.0f;

    float mot_aff = MotionAffinity(params_.motion_affinity_w, obj1.rect, obj2.rect);
    if (mot_aff < eps)
        return 0.0f;
    float time_aff =
        TimeAffinity(params_.time_affinity_w, static_cast<float>(obj1.frame_idx), static_cast<float>(obj2.frame_idx));

    if (time_aff < eps)
        return 0.0f;

    float app_aff = 1.0f - distance_fast_->Compute(descriptor1, descriptor2);

    return shp_aff * mot_aff * app_aff * time_aff;
}

float PedestrianTracker::Affinity(const TrackedObject& obj1, const TrackedObject& obj2) {
    float shp_aff = ShapeAffinity(params_.shape_affinity_w, obj1.rect, obj2.rect);
    float mot_aff = MotionAffinity(params_.motion_affinity_w, obj1.rect, obj2.rect);
    float time_aff =
        TimeAffinity(params_.time_affinity_w, static_cast<float>(obj1.frame_idx), static_cast<float>(obj2.frame_idx));
    return shp_aff * mot_aff * time_aff;
}

bool PedestrianTracker::IsTrackValid(size_t id) const {
    const auto& track = tracks_.at(id);
    const auto& objects = track.objects;
    if (objects.empty()) {
        return false;
    }
    int64_t duration_ms = objects.back().timestamp - track.first_object.timestamp;
    if (duration_ms < static_cast<int64_t>(params_.min_track_duration))
        return false;
    return true;
}

bool PedestrianTracker::IsTrackForgotten(size_t id) const {
    return IsTrackForgotten(tracks_.at(id));
}

bool PedestrianTracker::IsTrackForgotten(const Track& track) const {
    return (track.lost > params_.forget_delay);
}

std::unordered_map<size_t, std::vector<cv::Point>> PedestrianTracker::GetActiveTracks() const {
    std::unordered_map<size_t, std::vector<cv::Point>> active_tracks;
    for (size_t idx : active_track_ids()) {
        auto track = tracks().at(idx);
        if (IsTrackValid(idx) && !IsTrackForgotten(idx)) {
            active_tracks.emplace(idx, Centers(track.objects));
        }
    }
    return active_tracks;
}

TrackedObjects PedestrianTracker::TrackedDetections() const {
    TrackedObjects detections;
    for (size_t idx : active_track_ids()) {
        auto track = tracks().at(idx);
        if (IsTrackValid(idx) && !track.lost) {
            detections.emplace_back(track.objects.back());
        }
    }
    return detections;
}

cv::Mat PedestrianTracker::DrawActiveTracks(const cv::Mat& frame) {
    cv::Mat out_frame = frame.clone();

    if (colors_.empty()) {
        int num_colors = 100;
        colors_ = GenRandomColors(num_colors);
    }

    auto active_tracks = GetActiveTracks();
    for (auto active_track : active_tracks) {
        size_t idx = active_track.first;
        auto centers = active_track.second;
        DrawPolyline(centers, colors_[idx % colors_.size()], &out_frame);
        std::stringstream ss;
        ss << idx;
        putHighlightedText(out_frame,
                           ss.str(),
                           centers.back(),
                           cv::FONT_HERSHEY_SCRIPT_COMPLEX,
                           0.95,
                           colors_[idx % colors_.size()],
                           2);
        auto track = tracks().at(idx);
        if (track.lost) {
            cv::line(out_frame, active_track.second.back(), Center(track.predicted_rect), cv::Scalar(0, 0, 0), 4);
        }
    }

    return out_frame;
}
