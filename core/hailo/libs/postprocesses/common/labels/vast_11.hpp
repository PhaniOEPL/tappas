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
{2, "cab_trailer"},
        {3, "car"},
        {4, "fire_extinguisher"},
        {5, "forklift"},
{6, "new_forklift"},
        {7, "person"},
        {8, "safety_helmet"},
        {9, "safety_jacket"},
        {10, "towed_trolley"},
        {11, "tractor"},
        {12, "truck"}};
}
