/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: grid_map_envrionment.h
*   Author  : lubing.han
*   Date    : 2017-09-04
*   Describe:
*
********************************************************/
#include "coordinates_trans.h"
#include "roadmap.h"
#include "road_fun.h"
#include "grid_filler.h"
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>
#include <autodrive_msgs/CarStatus.h>
#include <math.h>
#include <ros/ros.h>

#ifndef GRID_MAP_ENVRIONMENT_H
#define GRID_MAP_ENVRIONMENT_H
#define GRID_MAP_WIDTH 500
#define GRID_MAP_HEIGHT 500
#define GRID_MAP_RESOLUTION 0.1

namespace opendrive {

class GridMapEnvironment {

public:
	GridMapEnvironment();
	~GridMapEnvironment() {}
	nav_msgs::OccupancyGrid& getGrid() {return _grid;}
	void update();
	bool isInitialized() const {return _initialized;}

private:
	void carStatusCallback(const autodrive_msgs::CarStatus& msg);

	ros::Subscriber _subCar;
	autodrive_msgs::CarStatus _carStatus;
	bool _initialized;
	const RoadMap* _roadmap;
	CoordinatesTrans* _trans;
	nav_msgs::OccupancyGrid _grid;
};

}
#endif
