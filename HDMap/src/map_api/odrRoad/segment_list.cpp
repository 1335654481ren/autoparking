/********************************************************
*   Copyright (C) 2018 All rights reserved.
*   
*   Filename:segment_list.cpp
*   Author  :lubing.han
*   Date    :2018-01-18
*   Describe:
*
********************************************************/
#include "segment_list.h"
#ifndef DEROS
#include <ros/ros.h>
#else
#include "ros_ros.h"
#endif
using namespace std;

namespace opendrive
{

void SegmentList::appendSegments(const vector<object::shape::Point>& points, bool closed) {
	if (points.size() < 1) return;
	for (int i = 0; i < points.size() - 1; ++i) {
		if (points[i].hasSegment())
			_segments.emplace_back(points[i], points[i + 1]);
		else
			_segments.emplace_back(points[i], points[i]);
	}
	if (closed || points.size() == 1) {
		if (points.back().hasSegment())
			_segments.emplace_back(points.back(), points.front());
		else
			_segments.emplace_back(points.back(), points.back());
	}
}

visualization_msgs::Marker SegmentList::toMarkerMsg() const {
	visualization_msgs::Marker marker;
	marker.header.stamp = ros::Time::now();
	marker.header.frame_id = "/map";
	marker.ns = "SegmentList";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = 0.1;
	for (auto&i : _segments) {
		if (i.first.hasSegment() && i.first.getSegment().getStyle().getType() == "none") continue;
		geometry_msgs::Point p;
		std_msgs::ColorRGBA c;
		p.x = i.first.getX();
		p.y = i.first.getY();
		p.z = i.first.getZ();
		marker.points.push_back(p);
		p.x = i.second.getX();
		p.y = i.second.getY();
		p.z = i.second.getZ();
		marker.points.push_back(p);
		c.r = 0.0, c.g = 1.0, c.b = 0.0, c.a = 1.0;
		if (i.first.hasSegment() && i.first.getSegment().hasColor()) {
			c.r = float(i.first.getSegment().getColor().getR()) / 255.0;
			c.g = float(i.first.getSegment().getColor().getG()) / 255.0;
			c.b = float(i.first.getSegment().getColor().getB()) / 255.0;
		}
		marker.colors.push_back(c);
		marker.colors.push_back(c);
	}
	return marker;
}

autodrive_msgs::SegmentList SegmentList::toSegmentListMsg() const {
	autodrive_msgs::SegmentList msg;
	msg.id = _id;
	msg.type = _type;
	msg.subtype = _subtype;
	for (auto& i : _segments) {
		msg.starts.push_back(i.first.toMsg());
		msg.ends.push_back(i.second.toMsg());
	}
	return msg;
}

void SegmentList::print(string prefix, int level) const {
	cout << prefix << "Segment List " << _id << ", type: " << _type << ", subtype: " << _subtype << endl;
	for (int i = 0; i < _segments.size(); ++i)
		cout << prefix << "  segment " << i + 1 << ": (" << _segments[i].first.getX() << ", " << _segments[i].first.getY() << ", " << _segments[i].first.getZ()
				 << ") -> (" << _segments[i].second.getX() << ", " << _segments[i].second.getY() << ", " << _segments[i].second.getZ() << ")" << endl;
	cout << endl;				 
}

}
