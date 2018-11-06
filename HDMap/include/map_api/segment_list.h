/********************************************************
*   Copyright (C) 2018 All rights reserved.
*   
*   Filename:segment_list.h
*   Author  :lubing.han
*   Date    :2018-01-18
*   Describe:
*
********************************************************/
#ifndef OPENDRIVE_SEGMENT_LIST_H
#define OPENDRIVE_SEGMENT_LIST_H

#include <vector>
#ifndef DEROS
#include <visualization_msgs/Marker.h>
#include <autodrive_msgs/SegmentList.h>
#else
#include "visualization_msgs_Marker.h"
#include "autodrive_msgs_SegmentList.h"
#endif
#include "opendrive_object_shape_point.h"
using namespace std;

namespace opendrive
{
class SegmentList
{
public:
	SegmentList() {};
	~SegmentList() {};

	int& getId() {return _id;}
	const int& getId() const {return _id;}
	void setId(const int &id) {_id = id;}

	string& getType() {return _type;}
	const string& getType() const {return _type;}
	void setType(const string &type) {_type = type;}

	string& getSubtype() {return _subtype;}
	const string& getSubtype() const {return _subtype;}
	void setSubtype(const string &subtype) {_subtype = subtype;}

	double& getHeight() {return _height;}
	const double& getHeight() const {return _height;}
	void setHeight(const double &height) {_height = height;}

	void clear() {_segments.clear();}
	void appendSegments(const vector<object::shape::Point>& points, bool closed = false);
	size_t size() const {return _segments.size();}
	const pair<object::shape::Point, object::shape::Point>& operator[](int n) {return _segments[n];}
	visualization_msgs::Marker toMarkerMsg() const;
	vector<pair<object::shape::Point, object::shape::Point> >& getSegments() {return _segments;}
	autodrive_msgs::SegmentList toSegmentListMsg() const;
	void setSegments(const vector<pair<object::shape::Point, object::shape::Point> >& segments) {_segments = segments;}
	void print(string prefix = "", int level = 10) const;
private:
	int _id = -1;
	string _type;
	string _subtype;
	double _height;
	vector<pair<object::shape::Point, object::shape::Point> > _segments;
};
}
#endif
