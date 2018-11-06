/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: coordinates_trans.h
*   Author  : lubing.han
*   Date    : 2017-03-22
*   Describe:
*
********************************************************/

#ifndef COORDINAGES_TRANS_H
#define COORDINAGES_TRANS_H

#include "lane_node.h"
#include "road_fun.h"
#include "map_api.h"
#include <vector>
#include <map>
#include <set>
#include <math.h>
#include <iostream>
using namespace std;

namespace opendrive {

class ObsRoads
{
public:
	ObsRoads();
	~ObsRoads();
	bool find(const string& id);
	double get_dist(const string& id);
	Point get_point(const string& id);
	void append(string id, double d, const Point& p);
private:
	map<string, double> _dists;
	map<string, Point> _points;
};

class CoordinatesTrans
{
public:
	static CoordinatesTrans* getInstance();
	~CoordinatesTrans();
	void update();
	double distFromRoad(const Point& p, string roadId);
	void xyz2sli(int obsId, Point &p, const vector<GlobalLaneId> &laneIds, GlobalLaneId &result); // not save result while obsId <= -10
	vector<const Road*> getNearRoads(const Point& p, double halfRange, double margin = 20.0);
	vector<const Object*> getNearObjects(const Point& p, double halfRange, double margin = 20.0);
private:
	CoordinatesTrans();
	static CoordinatesTrans* _trans;
	map<int, ObsRoads> _obs;
	map<string, LaneNode> _laneMap;
	const RoadMap* _roadMap;
};

}

#endif
