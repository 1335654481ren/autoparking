/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename: random_route_generator.cpp
*   Author  : lubing.han
*   Date    : 2017-12-27
*   Describe: 
*
********************************************************/
#include "random_route_generator.h"

namespace opendrive {

RandomRouteGenerator::RandomRouteGenerator() {
	srand(time(NULL));
	cout << "RandomRouteGenerator initializing routes ..." << endl;
	_roadMap = RoadMap::get_instance();
	vector<string> endRoads;
	for (const Road* road = _roadMap->front(); road != NULL; road = _roadMap->next(road)) {
		if (road->get_predecessors().size() > 0 && road->get_successors().size() > 0) continue;
		endRoads.push_back(road->get_id());
	}
	cout << "RandomRouteGenerator calculating routes from " << endRoads.size() * endRoads.size() << " posibilities" << endl;
	int count = 0;
	for (int i = 0; i < endRoads.size(); ++i) {
		for (int j = 0; j < endRoads.size(); ++j, ++count) {
			if (count % 10 == 0) cout << "RandomRouteGenerator: processing " << count << "th route" << endl;
			if (i == j) continue;
			if ((*_roadMap)[endRoads[i]]->get_length() < 10.0 || (*_roadMap)[endRoads[j]]->get_length() < 10.0) continue;
			vector<string> route = _router.getRoute(GlobalLaneId(endRoads[i], -1, 0), GlobalLaneId(endRoads[j], -1, 0));
			if (route.size() > 0) _routes.push_back(route);
			addEndPoint(endRoads[i]);
			addEndPoint(endRoads[j]);
		}
	}
	cout << "RandomRouteGenerator finished processing, totally " << _routes.size() << " routes" << endl;
}

RandomRouteGenerator::~RandomRouteGenerator() {

}

void RandomRouteGenerator::addEndPoint(const string& roadId) {
	if (_endPoints.find(roadId) != _endPoints.end()) return;
	const Road* road = (*_roadMap)[roadId];
	EndPoint endPoint;
	endPoint.roadId = roadId;
	const LaneSection* section;
	int flag;
	double s;
	if (road->get_predecessors().size() == 0) {
		section = road->front();
		s = 5.0;
		flag = 1;
	}
	else {
		section = road->back();
		s = road->get_length() - 5.0;
		flag = -1;
	}
	for (const Lane* lane = section->front(); lane != NULL; lane = section->next(lane)) {
		if (lane->get_id() == 0) continue;
		if (lane->get_id() * Drive_Direction * flag < 0)
			endPoint.startPoints.push_back(s2center_point(lane, s));
		else
			endPoint.endPoints.push_back(s2center_point(lane, s));
	}
	_endPoints[roadId] = endPoint;
}

bool RandomRouteGenerator::getRandomRoutePoints(Point& startPoint, Point& endPoint) const {
	if (_routes.size() == 0) return false;
	for (int i = 0; i < 100; ++i) {
		int n = rand() % _routes.size();
		Point p1, p2;
		if (getRandomPoint(_routes[n].front(), true, p1) && getRandomPoint(_routes[n].back(), false, p2)) {
			if (_router.getRoute(p1, p2).size() > 0) {
				startPoint = p1;
				endPoint = p2;
				return true;
			}
		}
	}
	return false;
}

bool RandomRouteGenerator::getRandomPoint(const string& roadId, bool isStart, Point& point) const {
	const EndPoint& endPoint = _endPoints.find(roadId)->second;
	int n = isStart ? endPoint.startPoints.size() : endPoint.endPoints.size();
	if (n == 0) return false;
	int i = rand() % n;
	point = isStart ? endPoint.startPoints[i] : endPoint.endPoints[i];
	return true;
}

} // namespace opendrive