/**
* Copyright (c) 2021-2022 Hailo Technologies Ltd. All rights reserved.
* Distributed under the LGPL license (https://www.gnu.org/licenses/old-licenses/lgpl-2.1.txt)
**/
#pragma once
#include <map>
namespace common
{
    static std::map<uint8_t, std::string> vast = {
        {0, "unlabeled"},
        {1, "bus"},
        {2, "car"},
        {3, "fire_extinguisher"},
        {4, "forklift"},
        {5, "person"},
        {6, "safety_helmet"},
        {7, "safety_jacket"},
        {8, "towed_trolley"},
        {9, "tractor"},
        {10, "truck"}};
}



