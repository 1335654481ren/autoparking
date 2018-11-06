/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: lane_fun.h
*   Author  : lubing.han
*   Date    : 2017-03-18
*   Describe:
*
********************************************************/

#ifndef LANE_FUN_H
#define LANE_FUN_H

#include "Lane.h"
#include "map_api_parameters.h"
#include "opendrive_object_shape_point.h"
using namespace std;

namespace opendrive {

int find_index(const Lane* lane, double s);
Point s2inner_point(const Lane* lane, double s);
Point s2center_point(const Lane* lane, double s);
Point s2lane_point(const Lane* lane, double s);
Point sl2point(const Lane* lane, double s, double l);
Point sdl2point(const Lane* lane, double s, double dl);
bool checkLaneChange(const Lane* lane, double& s, string& direction);
vector<object::shape::Point> lanePoints2shapePoints(const Lane* lane);

}

#endif