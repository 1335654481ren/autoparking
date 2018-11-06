/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: envlane_fun.h
*   Author  : lubing.han
*   Date    : 2017-07-06
*   Describe:
*
********************************************************/

#ifndef ENVLANE_FUN_H
#define ENVLANE_FUN_H

#include "map_api.h"
#include "grid.h"
#include "lane_converter.h"
#include <math.h>
#ifndef DEROS
#include <nav_msgs/OccupancyGrid.h>
#else
#include "nav_msgs_OccupancyGrid.h"
#endif
using namespace std;

#define ADD_OBS_EDGE 0

namespace opendrive {

void get_center_line(const RoadMap* roadmap, const EnvLane* envLane, vector<Point> &points);

void get_border_lines(const RoadMap* roadmap, const EnvLane* envLane,
	vector<Point> &points1, vector<Point> &points2);

vector<GlobalLaneId> get_local_lanes(const map<int, EnvLane>& envLanes, double minS, double maxS);

void fillLaneGrid(const EnvLane& envLane, nav_msgs::OccupancyGrid& grid, double (*eval)(double l));

void fillLaneGrid(const EnvLane& envLane, planning_matrix::Grid& grid, double (*eval)(double l));

void fillMapGrid(const map<int, EnvLane>& envLanes, const Point& carPoint, nav_msgs::OccupancyGrid& grid, string parkingId);

void fillMapGrid(const map<int, EnvLane>& envLanes, const Point& carPoint, planning_matrix::Grid& grid, string parkingId);

}

#endif
