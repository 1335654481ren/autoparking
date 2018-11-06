/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: envmarkers.cpp
*   Author  : lubing.han
*   Date    : 2016-07-07
*   Describe:
*
********************************************************/

#include "envmarkers.h"
#include "lane_fun.h"
#include "lane_converter.h"
#include <vector>
using namespace std;

namespace opendrive {

MarkerBase::MarkerBase(const Lane* lane, const Marker& marker, const MapInfo& mapInfo) {
	set_s(mapInfo.map2envS(marker.get_s()));
	set_id(marker.get_id());
	const vector<CornerRoad> &cornerRoads = marker.get_outline().get_cornerRoads();
	for (int i = 0; i < cornerRoads.size(); ++i) {
		Point p = sl2point(lane, cornerRoads[i].get_s(), cornerRoads[i].get_t());
		mapInfo.get_envSLT(p);
		_cornerPoints.push_back(p);
	}
	// cout << marker.get_type() << " "; print();
}

void MarkerBase::print() {
	cout << _id << ", s: " << _s << endl;
}

StopLine::StopLine(const Lane* lane, const Marker& marker, const MapInfo& mapInfo):
	MarkerBase(lane, marker, mapInfo) {
	const vector<Point> &points = get_cornerPoints();
	if (points.size() != 2) {
		ROS_ERROR("Wrong stopLine: size of cornerRoads does not equal to 2");
		return;
	}
	set_signalId(marker.get_refId(mapInfo.globalLaneId.laneId));
}

CrossWalk::CrossWalk(const Lane* lane, const Marker& marker, const MapInfo& mapInfo):
	MarkerBase(lane, marker, mapInfo) {
	const vector<Point> &points = get_cornerPoints();
	double minS = 1e10, maxS = -1e10;
	for (int i = 0; i < points.size(); ++i) {
		if (points[i].s > maxS) maxS = points[i].s;
		if (points[i].s < minS) minS = points[i].s;
	}
	set_maxS(maxS);
	set_minS(minS);
}

ParkingSpace::ParkingSpace(const Lane* lane, const Marker& marker, const MapInfo& mapInfo):
	MarkerBase(lane, marker, mapInfo) {
	double sums = 0.0, theta;
	Point p;
	const vector<Point> &points = get_cornerPoints();
	if (points.size() != 4) {
		ROS_ERROR("Wrong parkingSpace: size of cornerRoads does not equal to 4");
		return;
	}
	for (int i = 0; i < points.size(); ++i) {
		sums += points[i].s;
		p = p + points[i];
	}
	set_s(sums / points.size());

	if (points[0].dist2(points[1]) > points[1].dist2(points[2])) {
		theta = (points[1] - points[0]).yaw();
		_length = (sqrt(points[0].dist2(points[1])) + sqrt(points[2].dist2(points[3]))) / 2;
		_width = (sqrt(points[1].dist2(points[2])) + sqrt(points[3].dist2(points[0]))) / 2;
	}
	else {
		theta = (points[1] - points[2]).yaw();
		_width = (sqrt(points[0].dist2(points[1])) + sqrt(points[2].dist2(points[3]))) / 2;
		_length = (sqrt(points[1].dist2(points[2])) + sqrt(points[3].dist2(points[0]))) / 2;
	}
	double dt = points[0].theta - theta;
	while (dt > M_PI) dt -= 2 * M_PI;
	while (dt < -M_PI) dt += 2 * M_PI;
	if (dt < -0.8 * M_PI || dt > 0.8 * M_PI) theta += M_PI;
	else if (dt < -0.2 && p.l > 0) theta += M_PI;
	else if (dt > 0.2 && p.l < 0) theta += M_PI;
	// cout << p.x/4 << " " << p.y/4 << " " << p.z/4 << " " << theta << endl;
	_pose.position.x = p.x / 4, _pose.position.y = p.y / 4, _pose.position.z = p.y / 4;
	_pose.orientation = tf::createQuaternionMsgFromYaw(theta);
}

bool ParkingSpace::in_parkingSpace(const geometry_msgs::Point& point) {
	double dx = point.x - _pose.position.x;
	double dy = point.y - _pose.position.y;
	double theta = tf::getYaw(_pose.orientation);
	if (fabs(dx * cos(theta) + dy * sin(theta)) > _length / 2) return false;
	if (fabs(-dx * sin(theta) + dy * cos(theta)) > _width / 2) return false;
	return true;
}

NormalObject::NormalObject(const Lane* lane, const Marker& marker, const MapInfo& mapInfo):
	MarkerBase(lane, marker, mapInfo) {
	double sums = 0.0;
	const vector<Point> &points = get_cornerPoints();
	if (points.size() < 2) {
		ROS_ERROR("Wrong normalObject: size of cornerRoads smaller than 2");
		return;
	}
	for (int i = 0; i < points.size(); ++i)
		sums += points[i].s;
	set_s(sums / points.size());
}

}