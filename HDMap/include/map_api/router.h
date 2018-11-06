/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: router.h
*   Author  : lubing.han
*   Date    : 2017-03-06
*   Describe:
*
********************************************************/
#ifndef PLANNING_ROUTER_H
#define PLANNING_ROUTER_H

#include "map_api.h"
#include "roadmap_fun.h"
#include "coordinates_trans.h"
#include <map>
#ifndef DEROS
#include <ros/ros.h>
#else
#include "ros_ros.h"
#endif
#include <vector>
#include <string>
using namespace std;

namespace opendrive {

class RouterNode
{
public:
	RouterNode();
	RouterNode(const GlobalLaneId &id);
	void init();
	~RouterNode();
	double dist(const RouterNode* other);
	GlobalLaneId get_id();

	GlobalLaneId globalId;
	double len;

	bool visited;
	bool closed;
	double s;
	RouterNode* last;
	vector<RouterNode*> nexts;
};

class Router
{
public:
	Router();
	~Router();
	vector<string> getRoute(Point startP, Point endP) const;
	vector<string> getRoute(const GlobalLaneId& fromId, const GlobalLaneId& toId,
		double startS = -1.0, bool enableLaneChange = true, double endS = -1.0) const;

private:
	vector<GlobalLaneId> getLanes(const GlobalLaneId& laneId, double s, bool enableLaneChange, bool isStart) const;
	map<GlobalLaneId, RouterNode> constructGraph(
		const vector<GlobalLaneId>& startLanes, const vector<GlobalLaneId>& endLanes) const;
	vector<GlobalLaneId> shortestPath(RouterNode* startNode, RouterNode* endNode) const;

	const RoadMap* _roadMap;
};

}

#endif