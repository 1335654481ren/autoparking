/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename:Lane.h
*   Author  :weiwei.liu
*   Date    :2016-11-16
*   Describe:
*
********************************************************/

#ifndef HOBOT_LANE_H
#define HOBOT_LANE_H

#include "geometry.h"
#include "globallaneid.h"
#include <vector>
#include <string>
using namespace std;

namespace opendrive {

namespace RELATION {
enum RelationType
{
	NONE = 0,
	PREDECESSOR,
	SUCCESSOR,
	INNER_NEIGHBOUR,
	OUTER_NEIGHBOUR
};
}

class Polynomial {

public:
	Polynomial();

	// just for poly3
	Polynomial(double offset, double a, double b, double c, double d); 

	// just for poly 0
	Polynomial(double offset, double a);

	~Polynomial();

	double get_offset() const {return _offset;}
	void set_offset(double offset) {_offset = offset;}
	void push_param(double param) {_params.push_back(param);}
	void clear_param() {_params.clear();}
	double get_valueof(double x) const;
private:
	std::vector<double> _params;
	double _offset;
};

class RoadMark{

public:
	RoadMark();
	~RoadMark();
	void set_sOffset(const double sOffset) {_sOffset = sOffset;}
	void set_type(const string type) {_type = type;}
	void set_weight(const string weight) {_weight = weight;}
	void set_color(const string color) {_color = color;}
	void set_material(const string material) {_material = material;}
	void set_width(const double width) {_width = width;}
	void set_laneChange(const string laneChange) {_laneChange = laneChange;}
	void set_height(const double height) {_height = height;}
	double get_sOffset() const {return _sOffset;}
	string get_type() const {return _type;}
	string get_weight() const {return _weight;}
	string get_color() const {return _color;}
	string get_material() const {return _material;}
	double get_width() const {return _width;}
	string get_laneChange() const {return _laneChange;}
	double get_height() const {return _height;}

	bool checkLaneChange(const string& direction) const;

private:
	double _sOffset;
	string _type;
	string _weight;
	string _color;
	string _material;
	double _width;
	string _laneChange;
	double _height;
};

class Lane{

public:
	Lane(); 
	Lane(int id);
	~Lane();
	int get_id() const {return _id;}
	std::string get_type() const {return _type;}
	int get_predecessor() const {return _predecessor;}
	int get_successor() const {return _successor;}
	bool get_level() const {return _level;}
	double get_length() const {return _length;}
	double get_sOffset() const {return _sOffset;}
	double get_width(double s) const {return get_valueof(_width, s);}
	bool use_widths() const {return _width.size() > 0;}
	double get_border(double s) const {return get_valueof(_border, s);}
	double get_speed(double s) const {return get_valueof(_speed, s);}
	double get_innerheight(double s) const {return get_valueof(_innerheight, s);}
	double get_outerheight(double s) const {return get_valueof(_outerheight, s);}
	int get_roadmark_index(double s) const;
	const RoadMark get_roadmark(double s) const;
	const vector<RoadMark>& get_roadmarks() const {return _roadmarks;}
	const vector<Point>& get_lane_points() const {return _lane_points;}
	const vector<Point>& get_center() const {return _center_points;}
	const vector<Point>& get_inner_points() const {return _inner_points;}
	GlobalLaneId get_globalId() const {return _globalId;}
	const vector<GlobalLaneId>& get_predecessors() const {return _predecessors;}
	const vector<GlobalLaneId>& get_successors() const {return _successors;}
	const vector<GlobalLaneId>& get_inner_neighbour() const {return _inner_neighbour;}
	const vector<GlobalLaneId>& get_outer_neighbour() const {return _outer_neighbour;}
	RELATION::RelationType get_relation(const GlobalLaneId &globalid) const;

	void set_id(int id) {_id = id;}
	void set_type(const std::string& type) {_type = type;}
	void set_predecessor(int pre) {_predecessor = pre;}
	void set_successor(int suc) {_successor = suc;}
	void set_level(bool level ) {_level = level;}
	void set_length(double len) {_length = len;}
	void set_sOffset(double sOffset) {_sOffset = sOffset;}
	void append_width(Polynomial& poly) {_width.push_back(poly);}
	void append_border(Polynomial& poly) {_border.push_back(poly);}
	void append_speed(Polynomial& poly) {_speed.push_back(poly);}
	void append_innerheight(Polynomial& poly) {_innerheight.push_back(poly);}
	void append_outerheight(Polynomial& poly) {_outerheight.push_back(poly);}
	void append_roadmark(RoadMark& mark) {_roadmarks.push_back(mark);}
	void append_lane_point(Point& p) {_lane_points.push_back(p);}
	void append_center_point(Point& p) {_center_points.push_back(p);}
	void append_inner_point(Point& p) {_inner_points.push_back(p);}
	void set_globalId(string id, int sectionId, int laneId) {_globalId = GlobalLaneId(id, sectionId, laneId);}
	void append_predecessor(const GlobalLaneId &globalid);
	void append_successor(const GlobalLaneId &globalid);
	void append_inner_neighbour(const GlobalLaneId &globalid);
	void append_outer_neighbour(const GlobalLaneId &globalid);

private:
	double get_valueof(const std::vector<Polynomial>& vec, double s) const;

	int _id;
	double _sOffset;
	double _length;
	std::string _type;
	bool _level;
	int _predecessor;
	int _successor;
	
	std::vector<Polynomial> _width;
	std::vector<Polynomial> _border;
	std::vector<Polynomial> _speed;
	std::vector<Polynomial> _innerheight;
	std::vector<Polynomial> _outerheight;
	std::vector<RoadMark> _roadmarks;
	vector<Point> _lane_points;
	vector<Point> _center_points;
	vector<Point> _inner_points;
	GlobalLaneId _globalId;
	vector<GlobalLaneId> _predecessors;
	vector<GlobalLaneId> _successors;
	vector<GlobalLaneId> _inner_neighbour;
	vector<GlobalLaneId> _outer_neighbour;
};

} // end namespace opendrive

#endif


