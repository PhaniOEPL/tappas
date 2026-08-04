/**
* Copyright (c) 2021-2022 Hailo Technologies Ltd. All rights reserved.
* Distributed under the LGPL license (https://www.gnu.org/licenses/old-licenses/lgpl-2.1.txt)
**/
#pragma once

// General cpp includes
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

// Tappas includes
#include "strack.hpp"
#include "tracker_macros.hpp"


/**
 * @brief Create a cost matrix based on the features saved
 *        in each STrack. No return, is made, the matrix is
 *        filled in place.
 * 
 * @param tracks  -  std::vector<STrack*>
 *        Pointers to tracked STracks
 *
 * @param detections  -  std::vector<STrack>
 *        The newly detected STracks
 *
 * @param cost_matrix  -  std::vector<std::vector<float>>
 *        The cost matrix to fill in.
 *
 * @param cost_matrix_rows  -  int *
 *        To fill in with number of cost matrix rows.
 *
 * @param cost_matrix_cols  -  int *
 *        To fill in with number of cost matrix columns.
 */
inline void JDETracker::embedding_distance(std::vector<STrack*> &tracks,
                                           std::vector<STrack> &detections,
                                           std::vector<std::vector<float>> &cost_matrix)
{
    if (tracks.size() * detections.size() == 0)
    {
        return;
    }

    for (uint i = 0; i < tracks.size(); i++)
    {
        std::vector<float> cost_matrix_tmp(detections.size());
        std::vector<float> track_feature = tracks[i]->m_smooth_feat;
        for (uint j = 0; j < detections.size(); j++)
        {
            std::vector<float> det_feature = detections[j].m_curr_feat;
            float feat_square = 0.0;
            for (uint k = 0; k < det_feature.size(); k++)
            {
                feat_square += (track_feature[k] - det_feature[k])*(track_feature[k] - det_feature[k]);
            }
            cost_matrix_tmp[j] = std::sqrt(feat_square);
        }
        cost_matrix.push_back(cost_matrix_tmp);
    }
}


/**
 * @brief Update a cost matrix with the gating distance of all STracks.
 *        No returns are made 
 * 
 * @param cost_matrix  -  std::vector<std::vector<float>>
 *        A preliminary cost matrix made by embedding_distance
 *
 * @param tracks  -  std::vector<STrack*>
 *        Pointers to tracked STracks.
 *
 * @param detections  -  std::vector<STrack>
 *        The newly detected STracks.
 *
 * @param lambda_  -  float
 *        How much weight to give the gating distance.
 */
inline void JDETracker::fuse_motion(std::vector<std::vector<float>> &cost_matrix,
                                    std::vector<STrack*> &tracks,
                                    std::vector<STrack> &detections,
                                    float lambda_ = 0.98)
{
    if (cost_matrix.size() == 0)
        return;

    int gating_dim = 4;
    float gating_threshold = this->m_kalman_filter.chi2inv95[gating_dim];

    std::vector<TrackerTypes::DETECTBOX> measurements(detections.size());
    for (uint i = 0; i < detections.size(); i++)
    {
        std::vector<float> tlwh_ = detections[i].to_xyah();
        TrackerTypes::DETECTBOX measurement = {{tlwh_[0], tlwh_[1], tlwh_[2], tlwh_[3]}};
        measurements[i] = measurement;
    }

    for (uint i = 0; i < tracks.size(); i++)
    {
        xt::xarray<float, xt::layout_type::row_major> gating_distance = m_kalman_filter.gating_distance(tracks[i]->m_mean,
                                                                                                        tracks[i]->m_covariance,
                                                                                                        measurements);
        for (uint j = 0; j < cost_matrix[i].size(); j++)
        {
            if (gating_distance[j] > gating_threshold)
            {
                cost_matrix[i][j] = FLT_MAX;
            }
            cost_matrix[i][j] = lambda_ * cost_matrix[i][j] + (1 - lambda_)*gating_distance[j];
        }
    }
}

/**
 * @brief Apply both a class-consistency gate and a Kalman (Mahalanobis) motion
 *        gate to an IOU cost matrix, in place.
 *
 *        Two vetoes are applied (a match is forbidden by setting its cost to
 *        FLT_MAX, which is above any linear_assignment threshold):
 *          1) Class gate  - a track can never match a different-class detection.
 *          2) Motion gate - a detection whose squared Mahalanobis distance from
 *                           the track's predicted CENTER exceeds the chi-square
 *                           0.95 gate (2 DOF: x, y) is geometrically implausible
 *                           and is vetoed. Gating on the center only (not a, h)
 *                           avoids double-counting box size with the IOU cost.
 *
 *        The gates are vetoes only - the Mahalanobis distance is NOT blended
 *        into the cost value, so the matrix stays a pure IOU distance and the
 *        existing m_iou_thr / m_init_iou_thr thresholds keep their meaning.
 *
 *        The motion gate is skipped for tracks without a valid Kalman state
 *        (zero mean) - i.e. the unconfirmed "new" stracks in the step-4
 *        association, which are never activated/predicted. For those, only the
 *        class gate applies (identical to the previous behaviour).
 *
 * @param cost_matrix  -  std::vector<std::vector<float>>
 *        An IOU cost matrix (1 - IOU), modified in place.
 *
 * @param tracks  -  std::vector<STrack*>
 *        Pointers to the tracks (rows of the cost matrix).
 *
 * @param detections  -  std::vector<STrack>
 *        The newly detected STracks (columns of the cost matrix).
 *
 * @param gating_scale  -  float
 *        Multiplier on the chi-square motion gate. 1.0 is the textbook gate;
 *        larger values loosen it. The step-3.2 extended-IOU pass must pass a
 *        looser value (EXTENDED_IOU_GATING_SCALE) - that pass exists to recover
 *        large motion, so a textbook gate there would veto the very matches it
 *        is meant to rescue.
 */
inline void JDETracker::fuse_motion_custom(std::vector<std::vector<float>> &cost_matrix,
                                    std::vector<STrack*> &tracks,
                                    std::vector<STrack> &detections,
                                    float gating_scale)
{
    if (cost_matrix.size() == 0)
        return;

    // Motion gate threshold: 0.95 quantile of chi-square with 2 DOF (x, y),
    // widened by gating_scale. We gate on the box CENTER only; box size (a, h) is
    // left to the IOU cost, which already penalises size mismatch - gating on both
    // would count the same evidence twice.
    //
    // The scale is NOT 1.0 - see DEFAULT_GATING_SCALE in jde_tracker.hpp for why
    // this tracker's tight std weights make the textbook gate narrower than a
    // walking pace. Consider exposing the scale via the shm knobs (track_shmseg)
    // so it can move with the operator-selected motion profile.
    const int gating_dim = 2;
    const float gating_threshold = gating_scale * this->m_kalman_filter.chi2inv95[gating_dim];

    // Build the (x, y, a, h) measurement set from the current detections once.
    std::vector<TrackerTypes::DETECTBOX> measurements(detections.size());
    for (uint j = 0; j < detections.size(); j++)
    {
        std::vector<float> xyah = detections[j].to_xyah();
        TrackerTypes::DETECTBOX measurement = {{xyah[0], xyah[1], xyah[2], xyah[3]}};
        measurements[j] = measurement;
    }

    for (uint i = 0; i < tracks.size(); i++)
    {
        // A track only has a valid Kalman state once it has been activated.
        // Unconfirmed new stracks (step 4) still carry a zero mean/covariance,
        // for which gating_distance would divide by zero - apply class gate only.
        bool has_motion_state = ((float)xt::sum(tracks[i]->m_mean)() != 0.0f);

        xt::xarray<float> gating_distance;
        if (has_motion_state)
        {
            gating_distance = this->m_kalman_filter.gating_distance_xy(tracks[i]->m_mean,
                                                                       tracks[i]->m_covariance,
                                                                       measurements);
        }

        // Motion context for the direction-consistency cost.
        float velocity_x = 0.0f, velocity_y = 0.0f, speed = 0.0f;
        float anchor_x = 0.0f, anchor_y = 0.0f;
        float direction_weight = 0.0f;
        if (has_motion_state)
        {
            velocity_x = tracks[i]->m_mean(4);
            velocity_y = tracks[i]->m_mean(5);
            speed = std::sqrt((velocity_x * velocity_x) + (velocity_y * velocity_y));

            // multi_predict has ALREADY advanced the state this frame, so the
            // predicted centre is not a usable anchor - the displacement to it is
            // just the residual, which is ~0 for a well-tracked object and carries
            // no direction. Step back one velocity increment to recover where the
            // track was before the prediction, and measure displacement from there.
            anchor_x = tracks[i]->m_mean(0) - velocity_x;
            anchor_y = tracks[i]->m_mean(1) - velocity_y;

            // Ramp the penalty in by speed, in box-heights per frame.
            float height = tracks[i]->m_mean(3);
            if (height > 0.0f)
            {
                float speed_ratio = speed / height;
                direction_weight = (speed_ratio - DIRECTION_MIN_SPEED_RATIO) /
                                   (DIRECTION_FULL_SPEED_RATIO - DIRECTION_MIN_SPEED_RATIO);
                direction_weight = std::max(0.0f, std::min(1.0f, direction_weight));
            }
        }

        for (uint j = 0; j < cost_matrix[i].size(); j++)
        {
            // 1) Class gate - graded, not absolute.
            //    Unrelated classes are vetoed outright (a person track must never
            //    absorb a car detection). Classes the detector is KNOWN to confuse
            //    are only penalised, so a track with overwhelming geometric
            //    evidence can still claim the detection despite the label
            //    disagreeing. See class_policy.hpp.
            if (tracks[i]->m_class_id != detections[j].m_class_id)
            {
                if (!classes_are_confusable(tracks[i]->m_class_id, detections[j].m_class_id))
                {
                    cost_matrix[i][j] = FLT_MAX;
                    continue;
                }

                cost_matrix[i][j] += CLASS_MISMATCH_PENALTY;
            }

            // 2) Motion gate (only when the track has a valid Kalman state).
            if (has_motion_state && gating_distance[j] > gating_threshold)
            {
                cost_matrix[i][j] = FLT_MAX;
                continue;
            }

            // 3) Direction-consistency cost. Soft, speed-weighted, and applied
            //    only to matches that survived both gates. Penalises a candidate
            //    whose implied displacement contradicts the track's heading, which
            //    is what separates two same-class objects crossing when position
            //    and IOU have both gone uninformative. Note this only needs to
            //    make the WRONG pairing dearer than the right one - the assignment
            //    solver does the rest - so it breaks ties rather than vetoing.
            if (direction_weight > 0.0f && speed > 0.0f)
            {
                float dx = measurements[j](0, 0) - anchor_x;
                float dy = measurements[j](0, 1) - anchor_y;
                float displacement = std::sqrt((dx * dx) + (dy * dy));

                if (displacement > 0.0f)
                {
                    // +1 continues the heading, -1 reverses it.
                    float cosine = ((dx * velocity_x) + (dy * velocity_y)) / (displacement * speed);
                    // Remap to 0 (agrees) .. 1 (fully reversed).
                    float disagreement = (1.0f - cosine) * 0.5f;
                    cost_matrix[i][j] += DIRECTION_COST_WEIGHT * direction_weight * disagreement;
                }
            }
        }
    }
}
