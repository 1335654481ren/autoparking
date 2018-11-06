/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: coordinates_trans.cpp
*   Author  : lubing.han
*   Date    : 2017-03-02
*   Describe:
*
********************************************************/
#include "coordinates_trans.h"
#ifndef DEROS
#include <ros/ros.h>
#else
#include "ros_ros.h"
#endif
using namespace std;

namespace opendrive {

ObsRoads::ObsRoads() {
}

ObsRoads::~ObsRoads() {
}

bool ObsRoads::find(const string& id) {
	return _dists.find(id) != _dists.end();
}

double ObsRoads::get_dist(const string& id) {
	return _dists[id];
}

Point ObsRoads::get_point(const string& id) {
	return _points[id];
}

void ObsRoads::append(string id, double d, const Point& p) {
	_dists[id] = d;
	_points[id] = p;
}

CoordinatesTrans::CoordinatesTrans() {
	_roadMap = opendrive::RoadMap::get_instance();
	const Road* road = _roadMap->front();
	for ( ; road != NULL; road = _roadMap->next(road)) {
		string roadId = road->get_id();
		_laneMap[roadId] = LaneNode();
		_laneMap[roadId].set_points(road->get_refline());
		_laneMap[roadId].set_id(roadId);
	}
}

CoordinatesTrans::~CoordinatesTrans() {

}

CoordinatesTrans* CoordinatesTrans::_trans = NULL;
CoordinatesTrans* CoordinatesTrans::getInstance() {
	if (_trans == NULL)
		_trans = new CoordinatesTrans();
	return _trans;
}

void CoordinatesTrans::update() {
	_obs.clear();
}

double CoordinatesTrans::distFromRoad(const Point& p, string roadId) {
	LaneNode &laneNode = _laneMap[roadId];
	double d1 = 0, d2 = 1e10;
	while (d2 - d1 > 1e-5) laneNode.update_once(p, d1, d2);
	laneNode.reset();
	return d1;
}

void CoordinatesTrans::xyz2sli(int obsId, Point &p, const vector<GlobalLaneId> &laneIds, GlobalLaneId &result) {
	if (_obs.find(obsId) == _obs.end()) _obs[obsId] = ObsRoads();
	ObsRoads& obsRoads = _obs[obsId];
	set<string> roadIds;
	for (vector<GlobalLaneId>::const_iterator i = laneIds.begin(); i != laneIds.end(); ++i)
		roadIds.insert(i->id);

	// initialize
	double minDist = 1e10;
	string minRoad;
	set<string>::iterator road_iter = roadIds.begin();
	for (; road_iter != roadIds.end(); ++road_iter) {
		if (obsRoads.find(*road_iter)) {
			double d = obsRoads.get_dist(*road_iter);
			if (d < minDist) {
				minDist = d;
				minRoad = *road_iter;
			}
		}
	}
	// push possible roads
	vector<LaneNode*> distMap;
	for (road_iter = roadIds.begin(); road_iter != roadIds.end(); ++road_iter) {
		if (!obsRoads.find(*road_iter)) {
			double d1, d2;
			LaneNode* pNode = &(_laneMap[*road_iter]);
			pNode->update_once(p, d1, d2);
			if (d1 - 10 > minDist) pNode->reset();
			else distMap.push_back(pNode);
		}
	}
	// get minimal dist
	for (int i = 0; i < distMap.size(); ++i) {
		Point cp = p;
		double d1 = distMap[i]->get_minDist(), d2 = distMap[i]->get_maxDist();
		while (d2 - d1 > 1e-5) distMap[i]->update_once(cp, d1, d2);
		distMap[i]->get_sl(cp);
		double d;
		const Lane* lane = sl2lane((*_roadMap)[distMap[i]->get_id()], cp.s, cp.l);
		if (!lane) d = 1e10;
		else {
			Point lane_point = s2center_point(lane, cp.s);
			d = sqrt(lane_point.dist2xy(p));
		}
		if (d < minDist) {
			minDist = d;
			minRoad = distMap[i]->get_id();
		}
		obsRoads.append(distMap[i]->get_id(), d, cp);
		distMap[i]->reset();
	}
	if (minDist > 9e9) {
		result = GlobalLaneId("", 0, 0);
		if (obsId < -9) _obs.erase(obsId);
		return;
	}
	p = obsRoads.get_point(minRoad);

	const Lane* lane = sl2lane((*_roadMap)[minRoad], p.s, p.l);
	if (lane) {
		result = lane->get_globalId();
		for (int i = 0; i < laneIds.size(); ++i)
			if (result == laneIds[i]) {
				if (obsId < -9) _obs.erase(obsId);
				return;
			}
	}
	result = GlobalLaneId("", 0, 0);
	if (obsId < -9) _obs.erase(obsId);
	return;
}

vector<const Road*> CoordinatesTrans::getNearRoads(const Point& p, double halfRange, double margin) {
	vector<const Road*> roads;
	for (auto &i : _laneMap) {
		LaneNode &laneNode = i.second;
		double d1 = 0, d2 = 1e10;
		while (d2 - d1 > margin) laneNode.update_once(p, d1, d2);
		if (d1 < halfRange) roads.push_back((*_roadMap)[i.first]);
		laneNode.reset();
	}
	return roads;
}

vector<const Object*> CoordinatesTrans::getNearObjects(const Point& p, double halfRange, double margin) {
	vector<const Object*> objects;
	vector<const Obstacle*> obstacles = _roadMap->get_obstacles();
	for (auto &i : obstacles) {
		auto points = i->getGroundPoints();
		for (auto &j : points) {
			if (fabs(p.x - j.getX()) < halfRange && fabs(p.y - j.getY()) < halfRange) {
				objects.push_back((const Object*)i);
				break;
			}
		}
	}
	return objects;
}

}