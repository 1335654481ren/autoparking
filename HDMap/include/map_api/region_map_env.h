/********************************************************
*   Copyright (C) 2018 All rights reserved.
*   
*   Filename: region_map_env.h
*   Author  : lubing.han
*   Date    : 2018-01-16
*   Describe:
*
********************************************************/

#ifndef REGION_MAP_ENV_H
#define REGION_MAP_ENV_H

#include "coordinates_trans.h"
#include "map_api.h"
#include "segment_list.h"
#include "lane_fun.h"
#include "grid_filler.h"
#ifndef DEROS
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#else
#include "geometry_msgs_Pose.h"
#include "tf_transform_datatypes.h"
#include "nav_msgs_OccupancyGrid.h"
#include "ros_ros.h"
#endif
#include <math.h>
#include <pthread.h>
using namespace std;

namespace opendrive {

class ShadowPolygon
{
public:
	ShadowPolygon(const Point& origin, double maxRange, const vector<geometry_msgs::Point>& polygon);
	~ShadowPolygon();
	double coverdBy(const ShadowPolygon& other) const;
	bool includeOrigin() const {return _includeOrigin;}
	const vector<geometry_msgs::Point>& getPolygon() const {return _polygon;}
private:
	bool _includeOrigin;
	double _startAngle;
	double _endAngle;
	vector<geometry_msgs::Point> _polygon;
};

class RegionMapEnv {

public:
	RegionMapEnv(double senseRange = 150.0, double updateStep = 100.0, double tolerance = 50.0, int width = 1000, int height = 1000, double resolution = 0.1);
	~RegionMapEnv();
	void setLanesOnly(bool b) {_lanesOnly = b;}
	bool initialize(const geometry_msgs::Pose& p);
    bool update(
            const geometry_msgs::Pose& p,
            bool block = true,
            const vector<vector<geometry_msgs::Point>>* custom_objects_ptr = NULL
    );
	const nav_msgs::OccupancyGrid& getFreeSpace() const {return _grid;}
	const nav_msgs::OccupancyGrid& getLanesGrid() const {return _grid;}
	const vector<SegmentList> getSegmentLists() const {return _segmentLists;}

private:
	void prepareGrid(const geometry_msgs::Pose& p, nav_msgs::OccupancyGrid& grid);
	void updateFreeSpace(const geometry_msgs::Pose& p, const vector<const Road*>& roads, const vector<const Object*>& objects);
    void updateFreeSpaceWithCustomObj(const geometry_msgs::Pose& p, const vector<vector<geometry_msgs::Point>>& objects);
	void updateLanesGrid(const geometry_msgs::Pose& p, const vector<const Road*>& roads, const vector<const Object*>& objects);
	void updateLineList(const geometry_msgs::Pose& p, const vector<const Road*>& roads, const vector<const Object*>& objects);
	vector<pair<object::shape::Point, object::shape::Point> > shadowSegments(
		const vector<pair<object::shape::Point, object::shape::Point> >& segments, const nav_msgs::OccupancyGrid& grid) const;
	void fillRoads(nav_msgs::OccupancyGrid& grid, const vector<const Road*>& roads);
	static void* updateCallback(void* selfPtr);

	double _senseRange;
	double _updateStep;
	double _tolerance;
	double _margin;
	double _totalRange;

	bool _initialized = false;
	bool _updating = false;
	bool _lanesOnly = true;
	geometry_msgs::Pose _lastPos, _curPos;
	pthread_mutex_t _updateMutex;
	pthread_t _updateThread;
	CoordinatesTrans* _trans;
	vector<const Road*> _roads;
	vector<const Object*> _objects;
	vector<SegmentList> _segmentLists;
	nav_msgs::OccupancyGrid _grid;
	const RoadMap* _roadmap;
	map<GlobalLaneId, int> _idMap;
};

}
#endif
