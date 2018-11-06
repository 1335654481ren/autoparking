/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: lane_offset.h
*   Author  : lubing.han
*   Date    : 2017-03-16
*   Describe:
*
********************************************************/
#ifndef LANE_OFFSET_H
#define LANE_OFFSET_H

using namespace std;

namespace opendrive
{

class LaneOffset
{
public:
	LaneOffset() {}
	LaneOffset(double s, double a, double b, double c, double d): _s(s), _a(a), _b(b), _c(c), _d(d) {}
	~LaneOffset() {}

	void set_s(const double s) {_s = s;}
	void set_a(const double a) {_a = a;}
	void set_b(const double b) {_b = b;}
	void set_c(const double c) {_c = c;}
	void set_d(const double d) {_d = d;}

	double get_s() const {return _s;}
	double get_a() const {return _a;}
	double get_b() const {return _b;}
	double get_c() const {return _c;}
	double get_d() const {return _d;}

	double get_offset(const double s) const {
		double ds = s - _s;
		return _a + _b * ds + _c * ds * ds + _d * ds * ds * ds;
	}

private:
	double _s;
	double _a;
	double _b;
	double _c;
	double _d;
};

}

#endif
