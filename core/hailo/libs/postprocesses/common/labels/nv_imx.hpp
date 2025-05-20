/**
* Copyright (c) 2021-2022 Hailo Technologies Ltd. All rights reserved.
* Distributed under the LGPL license (https://www.gnu.org/licenses/old-licenses/lgpl-2.1.txt)
**/
#pragma once
#include <map>
namespace common
{
    static std::map<uint8_t, std::string> nv_imx = {
        {0, "person"},
        {1, "cars"},
        {2, "vehicles"},
        {3, "tanks"},
        {4, "ships"},
        {5, "boatsandstreams"},
        {6, "birds"},
        {7, "monkeys"},
        {8, "animals"},
        {9, "drones"},
        {10, "flights"},
        {11, "fighterflights"},
        {12, "helicopters"}};
}
