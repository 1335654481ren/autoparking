/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: road_fun.h
*   Author  : lubing.han
*   Date    : 2017-03-21
*   Describe:
*
********************************************************/

#ifndef ROAD_FUN_H
#define ROAD_FUN_H

#include "section_fun.h"
#include "road.h"
#ifndef DEROS
#include <geometry_msgs/Point.h>
#else
#include "geometry_msgs_Point.h"
#endif

using namespace std;

namespace opendrive {

class MarkerFilter
{
public:
	MarkerFilter() {}
	MarkerFilter(string t, bool r = false, double s = 0.0): type(t), reversed(r), startS(s) {}
	~MarkerFilter() {}
	
	string type;
	bool reversed;
	double startS;
};

const Lane* sl2lane(const Road* road, double s, double l);
bool sl2xyz(const Road* road, Point& p);
vector<Marker> get_markers(const Road* road, const GlobalLaneId &laneId, const MarkerFilter &filter);
GlobalLaneId get_nearest_lane(const Road* road, double s, const geometry_msgs::Point& p);

}

#endif



