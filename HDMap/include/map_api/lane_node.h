/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: lane_node.h
*   Author  : lubing.han
*   Date    : 2017-03-21
*   Describe:
*
********************************************************/
#ifndef LANE_NODE
#define LANE_NODE

#include "geometry.h"
#include "globallaneid.h"
#include <math.h>
#include <vector>
#include <iostream>
using namespace std;

namespace opendrive {

class LaneNode
{
public:
	LaneNode();
	~LaneNode();
	double get_minDist() const;
	double get_maxDist() const;
	string get_id() const;
	void set_id(const string id);
	void set_points(const vector<Point>& points);
	void set_points(const vector<Point>& points, int s, int e);
	void reset();
	void update_once(const Point &p, double& minDist, double& maxDist, double nowDist = 1e10);
	void get_sl(Point &p);
	bool isVisited() const;
	bool operator<(const LaneNode& other) const;
private:
	void clear();
	
	bool _visited;
	string _id;
	double _minDist, _maxDist;
	double _range;
	Point _p1, _p2;
	LaneNode *_lChild, *_rChild;
};

}

#endif