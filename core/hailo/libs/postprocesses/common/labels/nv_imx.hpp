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
}
