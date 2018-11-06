/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: junction_env.h
*   Author  : lubing.han
*   Date    : 2017-03-22
*   Describe:
*
********************************************************/

#ifndef JUNCTION_ENV_H
#define JUNCTION_ENV_H

#include "map_api.h"
#include "lane_converter.h"
#ifndef DEROS
#include <ros/ros.h>
#else
#include "ros_ros.h"
#endif

namespace opendrive {

class JunctionEnv
{
public:
	JunctionEnv() {}
	~JunctionEnv() {}
	void reset_env(int& globalEnvId, string junctionId,
		const vector<GlobalLaneId>& fromLanes, const vector<GlobalLaneId>& viaLanes, const vector<GlobalLaneId>& toLanes,
		const vector<int> fromGroups, const vector<int> toGroups);
	int get_mode(GlobalLaneId inLane, GlobalLaneId outLane) const;
	const EnvLane* get_envLane(GlobalLaneId inLane, GlobalLaneId outLane) const;
	const map<int, EnvLane>& get_junctionLanes() const {return _junctionLanes;}
	void get_conflictS(GlobalLaneId inLane, GlobalLaneId outLane, vector<int>& envIds, vector<double>& selfS, vector<double>& otherS) const;
	void print() const;

private:
	void generate_conflictS();
	double get_min_dist(Point p1, Point p2, Point q1, Point q2, double &t1, double &t2);

	const RoadMap* _roadMap;
	string _junctionId;
	map<int, EnvLane> _junctionLanes;
	vector<GlobalLaneId> _fromLanes, _viaLanes, _toLanes;
	vector<int> _fromGroups, _toGroups;
	vector<vector<double> > _conflictS;
};

}
#endif