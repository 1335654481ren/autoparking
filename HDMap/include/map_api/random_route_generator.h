/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename: random_route_generator.h
*   Author  : lubing.han
*   Date    : 2017-12-27
*   Describe: 
*
********************************************************/
#ifndef OPENDRIVE_RANDOM_ROUTE_GENERATOR_H
#define OPENDRIVE_RANDOM_ROUTE_GENERATOR_H

#include "roadmap.h"
#include "road_fun.h"
#include "lane_fun.h"
#include "router.h"
#include <time.h>
using namespace std;

namespace opendrive {

class EndPoint
{
public:
	EndPoint() {}
	~EndPoint() {}
	bool hasStartPoint() const {return startPoints.size() > 0;}
	bool hasEndPoint() const {return endPoints.size() > 0;}
	string roadId;
	vector<Point> startPoints, endPoints;
};

class RandomRouteGenerator
{
public:
	RandomRouteGenerator();
	~RandomRouteGenerator();
	bool getRandomRoutePoints(Point& startPoint, Point& endPoint) const;

private:
	void addEndPoint(const string& roadId);
	bool getRandomPoint(const string& roadId, bool isStart, Point& point) const;

	const RoadMap* _roadMap;
	Router _router;
	map<string, EndPoint> _endPoints;
	vector<vector<string> > _routes;
};

} // namespace opendrive

#endif