/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: section_fun.h
*   Author  : lubing.han
*   Date    : 2017-03-21
*   Describe:
*
********************************************************/

#ifndef SECTION_FUN_H
#define SECTION_FUN_H

#include "lane_fun.h"
#include "LaneSection.h"
using namespace std;

namespace opendrive {

vector<pair<double, bool> > get_laneChanges(const LaneSection* section, int laneId, bool outer = false, bool reversed = false);

}

#endif