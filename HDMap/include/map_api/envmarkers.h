/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: envmarkers.h
*   Author  : lubing.han
*   Date    : 2016-07-06
*   Describe:
*
********************************************************/

#ifndef ENVMARKERS_H
#define ENVMARKERS_H

#include "map_api.h"
#include <vector>
#ifndef DEROS
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#else
#include "geometry_msgs_Pose.h"
#include "ros_ros.h"
#include "tf_transform_datatypes.h"
#endif
using namespace std;

namespace opendrive {

class MapInfo;

class MarkerBase
{
public:
	MarkerBase() {}
	MarkerBase(const Lane* lane, const Marker& marker, const MapInfo& mapInfo);
	~MarkerBase() {}
	double get_s() const {return _s;}
	string get_id() const {return _id;}
	vector<Point>& get_cornerPoints() {return _cornerPoints;}
	const vector<Point>& get_cornerPoints() const {return _cornerPoints;}
	void set_s(double s) {_s = s;}
	void set_id(string id) {_id = id;}
	void append_cornerPoint(const Point& p) {_cornerPoints.push_back(p);}

	void print();
	virtual bool in_range(double s1, double s2) const {return s1 <= _s && _s <= s2;}

private:
	double _s;
	string _id;
	vector<Point> _cornerPoints;
};

class StopLine: public MarkerBase
{
public:
	StopLine() {}
	StopLine(const Lane* lane, const Marker& marker, const MapInfo& mapInfo);
	~StopLine() {}

	string get_signalId() const {return _signalId;}
	void set_signalId(string signalId) {_signalId = signalId;}
private:
	string _signalId;
};

class CrossWalk: public MarkerBase
{
public:
	CrossWalk() {}
	CrossWalk(const Lane* lane, const Marker& marker, const MapInfo& mapInfo);
	~CrossWalk() {}

	double get_maxS() const {return _maxS;}
	double get_minS() const {return _minS;}
	void set_maxS(double maxS) {_maxS = maxS;}
	void set_minS(double minS) {_minS = minS;}

	bool in_range(double s1, double s2) const {return s1 <= _minS && _maxS <= s2;}

private:
	double _maxS, _minS;
};

class ParkingSpace: public MarkerBase
{
public:
	ParkingSpace() {}
	ParkingSpace(const Lane* lane, const Marker& marker, const MapInfo& mapInfo);
	~ParkingSpace() {}

	geometry_msgs::Pose get_pose() const {return _pose;}
	double get_length() const {return _length;}
	double get_width() const {return _width;}
	void set_pose(const geometry_msgs::Pose& pose) {_pose = pose;}
	bool in_parkingSpace(const geometry_msgs::Point& point);
private:
	geometry_msgs::Pose _pose;
	double _length, _width;
};

class NormalObject: public MarkerBase
{
public:
	NormalObject() {}
	NormalObject(const Lane* lane, const Marker& marker, const MapInfo& mapInfo);
	~NormalObject() {}
};

}

#endif


