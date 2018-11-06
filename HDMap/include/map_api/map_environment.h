/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: map_environment.h
*   Author  : lubing.han
*   Date    : 2017-03-22
*   Describe:
*
********************************************************/

#ifndef MAP_ENVIRONMENT_H
#define MAP_ENVIRONMENT_H

#include "coordinates_trans.h"
#include "lane_converter.h"
#include "junction_env.h"
#include "map_api.h"
#include "grid_filler.h"
#include "envlane_fun.h"
#ifndef DEROS
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>
#include <autodrive_msgs/PlanningPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#else
#include "tf_transform_datatypes.h"
#include "nav_msgs_OccupancyGrid.h"
#include "autodrive_msgs_PlanningPoint.h"
#include "geometry_msgs_PoseStamped.h"
#include "ros_ros.h"
#endif
#include <math.h>

namespace opendrive {

class MapState
{
public:
	MapState(): curEnvId(0), curS(0.0), curRemain(0.0), outOfRoute(true){}
	~MapState() {}

	int curEnvId;
	GlobalLaneId curLaneId;
	double curS;
	double curRemain;
	bool outOfRoute;
	int laneChange;
};


class MapEnvironment {

public:
	MapEnvironment() {};
	MapEnvironment(int selfId = -1);
	const MapState* getMapState() const {return &_mapState;}
	const EnvLane* operator[](int id) const;
	const EnvLane* getCurEnvLane() const {return (*this)[_mapState.curEnvId];}
	const vector<EnvLane> operator[](const std::pair<char, int> tag) const;
	const map<int, EnvLane> get_envLanes() const {return _envLanes;}
	const vector<string> getRoute() const { return _route; }
	bool set_route(Point p);
	bool set_route(const vector<string>& route);
	bool set_parkingSpace(geometry_msgs::PoseStamped goal);
	bool update(const Point& p);

	void print(const map<int, EnvLane>& envLanes) const;

	// deprecated
	bool XYZ2SIL(Point &p, int obsId, int& envId) const;
	// deprecated
	Point mapSL2XYZ(string &roadId, double s, double l) const;
	// deprecated
	Point SI2XYZ(const GlobalLaneId &id, double s) const;
	// deprecated
	Point SIL2XYZ(const GlobalLaneId &id, double s, double l) const;

	double SI2L(double s, const EnvLane* envLane = NULL) const;
	Point SI2XYZ(double s, const EnvLane* envLane = NULL) const;
	// inner: dl < 0, outer: dl > 0
	Point SIdL2XYZ(double s, double dl, const EnvLane* envLane = NULL) const;
	vector<Point> SI2XYZ(const vector<double>& ss, const EnvLane* envLane = NULL) const;
	vector<Point> SIdL2XYZ(const vector<double>& ss, double dl, const EnvLane* envLane = NULL) const;
	vector<Point> SIdL2XYZ(const vector<double>& ss, const vector<double>& dls, const EnvLane* envLane = NULL) const;

	void getLongestRemain(int& envId, double &remain) const;
	void getConflictS(vector<int>& envIds, vector<double>& selfS, vector<double>& otherS);
	nav_msgs::OccupancyGrid& getGrid();

private:
	void updateCurState(const Point& p);
	void updateGrid(const Point& carPoint);
	vector<GlobalLaneId> get_laneIds() const;
	string get_self_junction(GlobalLaneId &inLane, GlobalLaneId &outLane) const;

	int _selfId;
	string _parkingId;
	
	map<int, EnvLane> _globalEnvLanes;
	map<GlobalLaneId, vector<MapInfo> > _globalMapLanes;

	map<int, EnvLane> _envLanes;
	map<GlobalLaneId, vector<MapInfo> > _mapLanes;

	map<string, JunctionEnv> _junctionEnvs;
	
	const RoadMap* _roadmap;
	vector<string> _route;
	vector<bool> _reversed;
	vector<double> _routeS;
	vector<GlobalLaneId> _route_lanes;
	vector<GlobalLaneId> _local_lanes;
	MapState _mapState;
	CoordinatesTrans* _trans;
	bool _line_end;
	nav_msgs::OccupancyGrid _grid;
};

}
#endif
