/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: roadmap.h
*   Author  : lubing.han
*   Date    : 2016-12-07
*   Describe:
*
********************************************************/
#ifndef ROADMAP_H
#define ROADMAP_H

#include "transformType.h"
#ifndef DEROS
#include "autodrive_msgs/Map.h"
#else
#include "autodrive_msgs_Map.h"
#endif
#include "globallaneid.h"
#include "junction.h"
#include "road.h"
#include "obstacle.h"
#include <map>
#include <string>
using namespace std;

namespace opendrive
{

class RoadMap;
bool process_doc(RoadMap &roadmap, string filepath);

class RoadMap
{
public:
	~RoadMap();
	static RoadMap* get_instance(string filename = "");
	
	const Road* operator[](const string &s) const;
	const Lane* operator[](const GlobalLaneId &globallanid) const;
	const Road* front() const;
	const Lane* front3() const;
	const Road* next(const Road* road) const;
	const Lane* next(const Lane* lane) const;
	
	bool init();
	bool append_road(Road& road);
	bool append_obstacle(Obstacle& obstacle);
	bool append_junction(const Junction &junction);
	void get_signal(const string signalId, const Signal* signal, string &roadId) const;
	const Junction* find_junction(const string id) const;
	const vector<Signal> get_signals() const;
	const vector<Signal> get_signals(const string road_id) const;
	vector<const Obstacle*> get_obstacles() const;

private:
	RoadMap();
	Road* operator[](const string &s);
	Lane* operator[](const GlobalLaneId &globallanid);
	Road* front();
	Lane* front3();
	Road* next(const Road* road);
	Lane* next(const Lane* lane);

	void init_road_relations();
	void init_lane_relations();

	static RoadMap* _proadmap;
	bool _initialized;
	map<string, Road> _roadmap;
	map<string, Junction> _junctionmap;
	map<string, Signal> _signalmap;
	map<string, string> _signalroad;
	map<string, Obstacle> _obstacles;
};

}

#endif
