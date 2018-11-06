/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: transformType.cpp
*   Author  : lubing.han
*   Date    : 2016-11-24
*   Describe:
*
********************************************************/

#include "transformType.h"
using namespace std;

namespace opendrive
{

autodrive_msgs::Point pointToMsg(const Point &point) {
	autodrive_msgs::Point p;
	p.x = point.x;
	p.y = point.y;
	p.z = point.z;
	p.s = point.s;
	p.theta = point.theta;
	return p;
}

autodrive_msgs::GlobalLaneId globalLaneIdToMsg(const GlobalLaneId &globalId) {
	autodrive_msgs::GlobalLaneId id;
	id.id = globalId.id;
	id.sectionId = globalId.sectionId;
	id.laneId = globalId.laneId;
	return id;
}

autodrive_msgs::RoadMark roadMarkToMsg(const RoadMark &roadMark) {
	autodrive_msgs::RoadMark roadMarkMsg;
	roadMarkMsg.sOffset = roadMark.get_sOffset();
	roadMarkMsg.type = roadMark.get_type();
	roadMarkMsg.weight = roadMark.get_weight();
	roadMarkMsg.color = roadMark.get_color();
	roadMarkMsg.material = roadMark.get_material();
	roadMarkMsg.width = roadMark.get_width();
	roadMarkMsg.laneChange = roadMark.get_laneChange();
	roadMarkMsg.height = roadMark.get_height();
	return roadMarkMsg;
}

autodrive_msgs::Lane laneToMsg(const Lane &lane) {
	autodrive_msgs::Lane l;
	const vector<Point> &points = lane.get_lane_points();
	l.globalid = globalLaneIdToMsg(lane.get_globalId());
	const vector<RoadMark>& roadMarks = lane.get_roadmarks();
	for (int i = 0; i < roadMarks.size(); ++i)
		l.roadMarks.push_back(roadMarkToMsg(roadMarks[i]));
	for (int i = 0; i < points.size(); ++i)
		l.points.push_back(pointToMsg(points[i]));
	return l;
}

autodrive_msgs::LaneSection laneSectionToMsg(LaneSection &laneScetion) {
	autodrive_msgs::LaneSection ls;
	const Lane* lane = laneScetion.front();
	while (lane)
		ls.lanes.push_back(laneToMsg(*lane));
	return ls;
}

autodrive_msgs::Road roadToMsg(Road &road) {
	autodrive_msgs::Road r;
	int lastSectionId = -1;
	Lane* lane = road.front()->front();
	while (lane) {
		if (lane->get_globalId().sectionId != lastSectionId) {
			lastSectionId = lane->get_globalId().sectionId;
			r.laneSections.push_back(autodrive_msgs::LaneSection());
		}
		r.laneSections.back().lanes.push_back(laneToMsg(*lane));
	}
	return r;
}

vector<autodrive_msgs::Point> laneToPointsMsg(const Lane &lane) {
	vector<autodrive_msgs::Point> ps;
	const vector<Point> &points = lane.get_lane_points();
	for (int i = 0; i < points.size(); ++i)
		ps.push_back(pointToMsg(points[i]));
	return ps;
}

visualization_msgs::Marker show_points(const vector<Point> &points, string ns, int id) {
    visualization_msgs::Marker m;
    m.header.frame_id = "/map";
    m.header.stamp = ros::Time::now();
    m.id = id; 
    m.ns = ns; 
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = 0.15;
    m.color.a = m.color.r = m.color.g = 0.9;
    m.pose.orientation.w = 1.0;
    for (int i = 0; i < points.size(); ++ i) {
        geometry_msgs::Point p;
        p.x = points[i].x;
        p.y = points[i].y;
        p.z = points[i].z + 0.5;
        m.points.push_back(p);
    }   
    return m;
}

visualization_msgs::Marker show_point(const Point &point, string ns, int id, double scale) {
    visualization_msgs::Marker m;
    m.header.frame_id = "/map";
    m.header.stamp = ros::Time::now();
    m.id = id; 
    m.ns = ns; 
    m.type = visualization_msgs::Marker::POINTS;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.color.a = m.color.r = m.color.g = m.color.b = 0.9;
    m.pose.orientation.w = 1.0;
    geometry_msgs::Point p;
    p.x = point.x;
    p.y = point.y;
    p.z = point.z + 0.01;
    m.points.push_back(p);
    return m;
}


}
