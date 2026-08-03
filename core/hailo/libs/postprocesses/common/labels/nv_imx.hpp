/**
* Copyright (c) 2021-2022 Hailo Technologies Ltd. All rights reserved.
* Distributed under the LGPL license (https://www.gnu.org/licenses/old-licenses/lgpl-2.1.txt)
**/
#pragma once
#include <map>
namespace common
{
    static std::map<uint8_t, std::string> nv_imx = {
        {0, "unlabeled"},
        {1, "person"},
        {2, "cars"},
        {3, "vehicles"},
        {4, "tanks"},
        {5, "ships"},
        {6, "boatsandstreams"},
        {7, "birds"},
        {8, "monkeys"},
        {9, "animals"},
        {10, "drones"},
        {11, "flights"},
        {12, "fighterflights"},
        {13, "helicopters"}};

    /**
     * Class-merge policy for the nv_imx taxonomy: source class id -> surviving class id.
     *
     * "cars" (2) and "vehicles" (3) are distinct at training time but semantically
     * nested - a car is also a vehicle - so the detector frequently alternates
     * between the two labels on the same object from frame to frame. The tracker
     * applies a hard class-consistency veto, so an alternating label means neither
     * track can ever match on the frames it loses: two tracks are created for one
     * object, each coasting on alternate frames, and because keep_tracked_frames
     * (5) is far longer than the 1-frame flicker period, neither ever expires.
     * The result is two permanently published boxes with two ids ping-ponging.
     *
     * Nothing downstream distinguishes them - the operator UI exposes a single
     * vehicle control that enables classes 2, 3 and 4 together - so the labels are
     * collapsed here, at the earliest point after decode. Merging (rather than
     * suppressing one box) is what removes the flicker: with a single label the
     * class veto can never fire, so the duplicate track is never created.
     *
     * Only add a pair here when the distinction is genuinely invisible downstream.
     * Note "tanks" (4) rides the same UI control but is visually distinct and is
     * deliberately NOT merged.
     */
    static std::map<uint32_t, uint32_t> nv_imx_class_merge = {
        {3, 2}}; // vehicles -> cars
}
