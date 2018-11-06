/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: roadmap_fun.h
*   Author  : lubing.han
*   Date    : 2017-05-03
*   Describe:
*
********************************************************/

#ifndef ROADMAP_FUN_H
#define ROADMAP_FUN_H

#include "roadmap.h"
#include "road_fun.h"
#include "lane_converter.h"
#include "junction_env.h"
#ifndef DEROS
#include <ros/ros.h>
#else
#include "ros_ros.h"
#endif
using namespace std;

namespace opendrive {

bool get_global_envlanes(const RoadMap* roadMap, const vector<string>& route,
	vector<bool>& reversed, vector<double>& routeS, map<int, EnvLane>& envLanes,
	map<GlobalLaneId, vector<MapInfo> >& mapLanes);

bool get_global_junctions(const RoadMap* roadMap, const map<int, EnvLane>& globalEnvLanes,
	map<string, JunctionEnv>& junctionLanes);

bool get_local_envlanes(const RoadMap* roadMap, const map<int, EnvLane>& globalEnvLanes,
	const map<string, JunctionEnv> junctionEnvs, map<int, EnvLane>& obsEnvLanes,
	map<GlobalLaneId, vector<MapInfo> >& obsMapLanes, GlobalLaneId curId, double curS);

// deprecated
void point2parkingSpaces(const RoadMap* roadMap, const geometry_msgs::Point &point,
	vector<geometry_msgs::Pose>& poses, vector<string>& ids);

// using parkings[0].get_pose() and parkings[0].get_length()
void point2parkingSpaces(const RoadMap* roadMap, const geometry_msgs::Point &point, vector<ParkingSpace>& parkings);

void point2parkingSpaces(const RoadMap* roadMap, const geometry_msgs::Point &point,
	vector<ParkingSpace>& parkings, vector<string>& roadIDs);

}

#endif
