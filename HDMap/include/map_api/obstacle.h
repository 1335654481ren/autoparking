/********************************************************
*   Copyright (C) 2018 All rights reserved.
*   
*   Filename:obstacle.h
*   Author  :lubing.han
*   Date    :2018-01-16
*   Describe:
*
********************************************************/
#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <vector>
#include "opendrive_object.h"
#include "geometry.h"
using namespace std;

namespace opendrive
{
class Obstacle: public Object
{

	
public:
	Obstacle() {};
	Obstacle(const Object& obj): Object(obj) {}
	~Obstacle() {};

	vector<object::shape::Point> getGroundPoints() const;
};
}
#endif
