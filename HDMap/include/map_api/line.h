/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename:line.h
*   Author  :siyu.wang
*   Date    :2016-11-14
*   Describe:
*
********************************************************/
#ifndef LINE_H
#define LINE_H

#include <vector>
#include "geometry.h"
using namespace std;

namespace opendrive
{
class Line: public Geometry
{

	
public:
	Line(){};
	Line(Point &Start,double Len,double Hdg);
	~Line(){};
	virtual void get_points(std::vector<Point>& points,double density);
	virtual Geometry* clone();

};
}
#endif
