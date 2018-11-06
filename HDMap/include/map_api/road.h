/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: road.h
*   Author  : lubing.han
*   Date    : 2016-11-16
*   Describe:
*
********************************************************/
#ifndef ROAD_H
#define ROAD_H

#include "signal.h"
#include "marker.h"
#include "elevation.h"
#include "planview.h"
#include "lane_offset.h"
#include "globallaneid.h"
#include <string>
#include "LaneSection.h"
#include "map_api_parameters.h"
#include <map>

using namespace std;
namespace opendrive
{
class Link
{
public:
	Link(): elementType(""), elementId(""), contactPoint("") {};
	Link(string type, string id, string cp): elementType(type), elementId(id), contactPoint(cp) {}
	~Link() {};

	string elementType;
	string elementId;
	string contactPoint;
};

class Road
{
public:
	Road() {};
	Road(double length, string id, string junction): _length(length), _id(id), _junction(junction) {};
	~Road() {};

	LaneSection* operator[](int i);
	Lane* operator[](const GlobalLaneId& globalId);
	LaneSection* front();
	Lane* front2();
	LaneSection* back();
	Lane* next(const Lane* lane);
	LaneSection* next(const LaneSection* section);
	const LaneSection* operator[](int i) const;
	const Lane* operator[](const GlobalLaneId& globalId) const ;
	const LaneSection* front() const;
	const Lane* front2() const;
	const LaneSection* back() const;
	const Lane* next(const Lane* lane) const;
	const LaneSection* next(const LaneSection* section) const;

	double get_length() const {return _length;}
	string get_id() const {return _id;}
	string get_junction() const {return _junction;}
	Link get_predecessor() const {return _predecessor;}
	Link get_successor() const {return _successor;}
	const vector<Link>& get_predecessors() const {return _predecessors;}
	const vector<Link>& get_successors() const {return _successors;}
	vector<LaneSection>& get_sections() {return _sections;}
	RELATION::RelationType get_relation(const string &id) const;
	int get_section_index(double s) const;
	const vector<Point>& get_refline() const {return _refline_points;}
	const double get_lane_offset(double s) const;
	const vector<LaneOffset>& get_lane_offsets() const {return _lane_offsets;}
	const vector<Signal>& get_signals() const {return _signals;}
	const vector<Marker>& get_markers() const {return _markers;}
	const vector<Elevation>& get_elevations() const {return _elevations;}
	const double get_elevation(double s) const;

	void set_predecessor(Link p) {_predecessor = p;}
	void set_successor(Link s) {_successor = s;}
	void append_predecessor(const Link &link);
	void append_successor(const Link &link);
	void append_geometry(Geometry* pgeo) {_refline.append_geometry(pgeo);}
	void append_section(LaneSection& sec) {_sections.push_back(sec);}
	void append_lane_offset(const LaneOffset &lane_offset) {_lane_offsets.push_back(lane_offset);}
	void append_signal(const Signal &signal) {_signals.push_back(signal);}
	void append_marker(const Marker &marker) {_markers.push_back(marker);}
	void append_elevation(const Elevation &elevation) {_elevations.push_back(elevation);}

	void init_points();
private:
	vector<vector<Point> > cutPoints(const vector<vector<Point> >& lanes_points);
	bool update_bounds(const Point& p1, const Point& p2, vector<double>& bounds);

	double _length;
	string _id;
	string _junction;

	Link _predecessor;
	Link _successor;
	vector<Link> _predecessors;
	vector<Link> _successors;
	PlanView _refline;
	vector<LaneOffset> _lane_offsets;
	vector<Point> _refline_points;
	vector<LaneSection> _sections;
	vector<Signal> _signals;
	vector<Marker> _markers;
	vector<Elevation> _elevations;
};

}

#endif
