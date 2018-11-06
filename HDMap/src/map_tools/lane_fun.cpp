/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: lane_fun.cpp
*   Author  : lubing.han
*   Date    : 2017-03-20
*   Describe:
*
********************************************************/

#include "lane_fun.h"
namespace opendrive{

Point get_middle(const Point& p1, const Point& p2, double t) {
	Point p = (1 - t) * p1 + t * p2;
	double t1 = p1.theta, t2 = p2.theta;
	while (t1 - t2 > M_PI) t1 -= 2 * M_PI;
	while (t2 - t1 > M_PI) t2 -= 2 * M_PI;
	p.theta = (1 - t) * t1 + t * t2;
	return p;
}

int find_index(const Lane* lane, double s) {
	const vector<Point>& points = lane->get_lane_points();
	int n = points.size();
	int lo = 0, hi = n;
	if (n == 0) return -1;
	while (hi - lo > 1) {
		int mi = (hi+lo)/2;
		if (points[mi].s <= s) lo = mi;
		else hi = mi;
	}
	return lo;
}

Point s2inner_point(const Lane* lane, double s) {
	int index = find_index(lane, s);
	const vector<Point>& points = lane->get_inner_points();
	if (index + 1 == points.size()) --index;
	double t = (s - points[index].s) / (points[index + 1].s - points[index].s);
	return get_middle(points[index], points[index + 1], t);
}

Point s2center_point(const Lane* lane, double s) {
	int index = find_index(lane, s);
	const vector<Point>& center = lane->get_center();
	if (index + 1 == center.size()) --index;
	double t = (s - center[index].s) / (center[index + 1].s - center[index].s);
	return get_middle(center[index], center[index + 1], t);
}

Point s2lane_point(const Lane* lane, double s) {
	int index = find_index(lane, s);
	const vector<Point>& points = lane->get_lane_points();
	if (index + 1 == points.size()) --index;
	double t = (s - points[index].s) / (points[index + 1].s - points[index].s);
	return get_middle(points[index], points[index + 1], t);
}

Point sl2point(const Lane* lane, double s, double l) {
	Point p = s2center_point(lane, s);
	double dl = l - p.l;
	p.x += -sin(p.theta) * dl;
	p.y += cos(p.theta) * dl;
	p.l = l;
	return p;
}

Point sdl2point(const Lane* lane, double s, double dl) {
	Point p = s2center_point(lane, s);
	p.x += -sin(p.theta) * dl;
	p.y += cos(p.theta) * dl;
	p.l += dl;
	return p;
}

bool checkLaneChange(const Lane* lane, double& s, string& direction) {
	if (lane->get_id() == 0) return false;
	int b = lane->get_roadmark_index(s);
	if (b < 0) return true;
	int i = b, e = -1, step = -1;
	const vector<RoadMark>& roadMarks = lane->get_roadmarks();
	if (lane->get_id() * Drive_Direction < 0) {
		e = roadMarks.size();
		step = 1;
	}
	for ( ; i != e; i += step)
		if (roadMarks[i].checkLaneChange(direction))
			break;
	if (i == e) return false;
	if (i == b) return true;
	if (lane->get_id() * Drive_Direction < 0)
		s = roadMarks[i].get_sOffset() + lane->get_sOffset();
	else
		s = roadMarks[i + 1].get_sOffset() + lane->get_sOffset();
	return true;
}

vector<object::shape::Point> lanePoints2shapePoints(const Lane* lane) {
	vector<object::shape::Point> shapePoints;
	const vector<Point>& lanePoints = lane->get_lane_points();
	auto& roadMarks = lane->get_roadmarks();
	vector<Point> roadMarkPoints;
	for (auto& i : roadMarks) roadMarkPoints.push_back(s2lane_point(lane, i.get_sOffset()));
	vector<const RoadMark*> mergeRoadmarks;
	vector<Point> mergePoints;
	int i = 0, j = 0;
	if (lanePoints.size() == 0 || roadMarkPoints.size() == 0) return shapePoints;
	while (i < lanePoints.size() || j < roadMarkPoints.size()) {
		if (i == j && j == 0 || i < lanePoints.size() && j < roadMarkPoints.size() && fabs(lanePoints[i].s - roadMarkPoints[j].s) < 0.1) {
			mergeRoadmarks.push_back(&(roadMarks[j]));
			mergePoints.push_back(roadMarkPoints[j]);
			++i, ++j;
		}
		else if (j == roadMarkPoints.size()) {
			mergeRoadmarks.push_back(&(roadMarks[j - 1]));
			mergePoints.push_back(lanePoints[i++]);
		}
		else if (i == lanePoints.size()) {
			mergeRoadmarks.push_back(&(roadMarks[j]));
			mergePoints.push_back(roadMarkPoints[j++]);
		}
		else if (lanePoints[i].s < roadMarkPoints[j].s) {
			mergeRoadmarks.push_back(&(roadMarks[j - 1]));
			mergePoints.push_back(lanePoints[i++]);
		}
		else {
			mergeRoadmarks.push_back(&(roadMarks[j]));
			mergePoints.push_back(roadMarkPoints[j++]);
		}
	}
	for (int k = 0; k < mergePoints.size(); ++k) {
		auto& lp = mergePoints[k];
		object::shape::Point sp;
		sp.setX(lp.x); sp.setY(lp.y); sp.setZ(lp.z);
		sp.setExact(false);
		sp.setHasSegment(true);
		auto& segment = sp.getSegment();
		auto& style = segment.getStyle();
		style.setType(mergeRoadmarks[k]->get_type());
		style.setWidth(mergeRoadmarks[k]->get_width());
		segment.setHasColor(true);
		segment.setHasRightColor(false);
		segment.setHasLeftColor(false);
		auto& color = segment.getColor();
		if (mergeRoadmarks[k]->get_color() == "yellow") {
			color.setR(255); color.setG(255); color.setB(0);
		}
		else {
			color.setR(255); color.setG(255); color.setB(255);
		}
		shapePoints.push_back(sp);
	}
	return shapePoints;
}

}