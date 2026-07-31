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
#include <set>
#include <string>
#include <vector>

// Tappas includes
#include "strack.hpp"
#include "tracker_macros.hpp"

// Macros
const float IOU_THRESHOLD = 0.15;


/**
 * @brief Returns a union of two vectors of STracks
 * 
 * @param tlista  -  std::vector<STrack *>
 *        A set of STracks to join (by pointer)
 *
 * @param tlistb  -  std::vector<STrack *>
 *        A set of STracks to join (by pointer)
 *
 * @return std::vector<STrack *> 
 *         Pointers to the union of the two sets
 */
inline std::vector<STrack *> JDETracker::joint_strack_pointers(std::vector<STrack *> &tlista, std::vector<STrack *> &tlistb)
{
    std::vector<STrack *> new_pool;
    new_pool.reserve( tlista.size() + tlistb.size() ); // preallocate memory
    new_pool.insert( new_pool.end(), tlista.begin(), tlista.end() );
    new_pool.insert( new_pool.end(), tlistb.begin(), tlistb.end() );
    return new_pool;
}


/**
 * @brief Returns a union of two vectors of STracks
 * 
 * @param tlista  -  std::vector<STrack>
 *        A set of STracks to join
 *
 * @param tlistb  -  std::vector<STrack>
 *        A set of STracks to join
 *
 * @return std::vector<STrack *> 
 *         Pointers to the union of the two sets
 */
inline std::vector<STrack *> JDETracker::joint_strack_pointers(std::vector<STrack> &tlista, std::vector<STrack> &tlistb)
{
    std::set<int> existing_stracks_set;
    std::vector<STrack *> res;
    for (uint i = 0; i < tlista.size(); i++)
    {
        existing_stracks_set.insert(tlista[i].m_track_id);
        res.push_back(&tlista[i]);
    }
    for (uint i = 0; i < tlistb.size(); i++)
    {
        int tid = tlistb[i].m_track_id;
        if (existing_stracks_set.count(tid) == 0)
        {
            existing_stracks_set.insert(tid);
            res.push_back(&tlistb[i]);
        }
    }
    return res;
}

/**
 * @brief Returns a union of two vectors of STracks
 * 
 * @param tlista  -  std::vector<STrack *>
 *        A set of STracks to join
 *
 * @param tlistb  -  std::vector<STrack>
 *        A set of STracks to join
 *
 * @return std::vector<STrack *> 
 *         A vector union of the two sets
 */
inline std::vector<STrack> JDETracker::joint_stracks(std::vector<STrack> &tlista, std::vector<STrack> &tlistb)
{
    std::set<int> existing_stracks_set;
    std::vector<STrack> res;
    for (uint i = 0; i < tlista.size(); i++)
    {
        existing_stracks_set.insert(tlista[i].m_track_id);
        res.push_back(tlista[i]);
    }
    for (uint i = 0; i < tlistb.size(); i++)
    {
        int tid = tlistb[i].m_track_id;
        if (existing_stracks_set.count(tid) == 0)
        {
            existing_stracks_set.insert(tid);
            res.push_back(tlistb[i]);
        }
    }
    return res;
}

/**
 * @brief Subtract one set of Stracks from another
 * 
 * @param tlista  -  std::vector<STrack>
 *        The STracks to subtract from
 *
 * @param tlistb  -  std::vector<STrack>
 *        The STracks to subtract from tlista
 *
 * @return std::vector<STrack> 
 *         A vector of STracks in tlista that don't appear tlistb.
 */
inline std::vector<STrack> JDETracker::sub_stracks(std::vector<STrack> &tlista, std::vector<STrack> &tlistb)
{
    std::map<int, STrack> stracks;
    for (uint i = 0; i < tlista.size(); i++)
    {
        stracks.insert(std::pair<int, STrack>(tlista[i].m_track_id, tlista[i]));
    }
    for (uint i = 0; i < tlistb.size(); i++)
    {
        int tid = tlistb[i].m_track_id;
        if (stracks.count(tid) != 0)
        {
            stracks.erase(tid);
        }
    }

    std::vector<STrack> res;
    for (auto &it : stracks)
    {
        res.push_back(it.second);
    }
    return res;
}

/**
 * @brief Perform a difference between two sets of STracks,
 *        Ending up with two sets of exclusive instances.
 *        No return is made, the vestors are changed in place.
 * 
 * @param resa  -  std::vector<STrack>
 *        A vector to fill in with STracks that are exclusive to stracksa
 *
 * @param resb  -  std::vector<STrack>
 *        A vector to fill in with STracks that are exclusive to stracksb
 *
 * @param stracksa  -  std::vector<STrack>
 *        A set of STracks, exclusive instances will end up in resa
 *
 * @param stracksb  -  std::vector<STrack>
 *        A set of STracks, exclusive instances will end up in resb
 */
inline void JDETracker::remove_duplicate_stracks(std::vector<STrack> &stracksa, std::vector<STrack> &stracksb)
{
    std::vector<STrack> resa, resb;
    std::vector<std::vector<float>> pdist = iou_distance(stracksa, stracksb);
    std::vector<std::pair<int, int>> pairs;
    for (uint i = 0; i < pdist.size(); i++)
    {
        for (uint j = 0; j < pdist[i].size(); j++)
        {
            // Never treat tracks of different classes as duplicates of each
            // other - the whole association path enforces class consistency,
            // and silently deleting a valid track of another class would be a
            // far worse failure than leaving two boxes overlapping.
            if (stracksa[i].m_class_id != stracksb[j].m_class_id)
                continue;

            if (pdist[i][j] < IOU_THRESHOLD)
            {
                pairs.push_back(std::pair<int, int>(i, j));
            }
        }
    }

    std::vector<int> dupa, dupb;
    for (uint i = 0; i < pairs.size(); i++)
    {
        STrack &tracka = stracksa[pairs[i].first];
        STrack &trackb = stracksb[pairs[i].second];

        // m_start_frame is only ever set by activate(), so a strack that has
        // never been activated still carries m_start_frame == 0 and would
        // compute an "age" equal to the whole frame counter - which would make
        // the newborn look older than a genuine long-lived track and get the
        // WRONG one deleted. Such a strack is by definition the younger of the
        // pair, so decide on activation before falling back to age.
        bool a_activated = (tracka.m_start_frame > 0);
        bool b_activated = (trackb.m_start_frame > 0);

        if (a_activated != b_activated)
        {
            if (a_activated)
                dupb.push_back(pairs[i].second);
            else
                dupa.push_back(pairs[i].first);
            continue;
        }

        int timep = tracka.m_frame_id - tracka.m_start_frame;
        int timeq = trackb.m_frame_id - trackb.m_start_frame;
        if (timep > timeq)
            dupb.push_back(pairs[i].second);
        else
            dupa.push_back(pairs[i].first);
    }

    for (uint i = 0; i < stracksa.size(); i++)
    {
        std::vector<int>::iterator iter = find(dupa.begin(), dupa.end(), i);
        if (iter == dupa.end())
        {
            resa.push_back(stracksa[i]);
        }
    }

    for (uint i = 0; i < stracksb.size(); i++)
    {
        std::vector<int>::iterator iter = find(dupb.begin(), dupb.end(), i);
        if (iter == dupb.end())
        {
            resb.push_back(stracksb[i]);
        }
    }

    // Remove the duplicates
    stracksa.clear();
    stracksa.assign(resa.begin(), resa.end());
    stracksb.clear();
    stracksb.assign(resb.begin(), resb.end());
}

/**
 * @brief Drop any new (unconfirmed) strack that sits on top of an already
 *        confirmed track of the same class.
 *
 *        This is the duplicate-id guard for the case remove_duplicate_stracks
 *        cannot handle: the new strack has never been activated, so it has no
 *        track id and no m_start_frame, and an age comparison against it is
 *        meaningless. No age test is needed here - a new strack is younger than
 *        a confirmed one by construction, so it always loses.
 *
 *        Only high-overlap pairs are removed (IOU > 1 - IOU_THRESHOLD, i.e.
 *        ~0.85). At that overlap the confirmed track's box still covers the
 *        object, so suppressing the newcomer costs no coverage - it only stops
 *        a second id being issued for something already being tracked.
 *
 * @param new_stracks  -  std::vector<STrack>
 *        The unconfirmed stracks, filtered in place.
 *
 * @param confirmed_stracks  -  std::vector<STrack>
 *        The confirmed tracks to test against. Pass the TRACKED set only; lost
 *        tracks are not published, so deduplicating against them would leave
 *        the object with no reported box.
 */
inline void JDETracker::remove_duplicate_new_stracks(std::vector<STrack> &new_stracks, std::vector<STrack> &confirmed_stracks)
{
    if (new_stracks.size() == 0 || confirmed_stracks.size() == 0)
        return;

    std::vector<std::vector<float>> pdist = iou_distance(new_stracks, confirmed_stracks);

    std::vector<STrack> kept;
    kept.reserve(new_stracks.size());

    for (uint i = 0; i < new_stracks.size(); i++)
    {
        bool is_duplicate = false;
        for (uint j = 0; j < confirmed_stracks.size(); j++)
        {
            if (new_stracks[i].m_class_id != confirmed_stracks[j].m_class_id)
                continue;

            if (pdist[i][j] < IOU_THRESHOLD)
            {
                is_duplicate = true;
                break;
            }
        }

        if (!is_duplicate)
            kept.push_back(new_stracks[i]);
    }

    new_stracks.clear();
    new_stracks.assign(kept.begin(), kept.end());
}
