/********************************************************
*   Copyright (C) 2016 All rights reserved.
*
*   Filename: grid_filler.h
*   Author  : lubing.han
*   Date    : 2017-07-05
*   Describe: 
*
********************************************************/
#ifndef GRID_FILLER_H
#define GRID_FILLER_H

#ifndef DEROS
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#else
#include "ros_ros.h"
#include "nav_msgs_OccupancyGrid.h"
#include "geometry_msgs_Point.h"
#endif
#include "grid.h"
using namespace std;

namespace opendrive {

#define COVER_ALL 0
#define COVER_IF_LARGER 1
#define COVER_IF_SMALLER 2
#define COVER_IF_NONNEG 3
#define COVER_IF_42 4
#define COVER_IF_NOT_42 5

double simplePiecewiseLinear(double d);

void fillTriangle(nav_msgs::OccupancyGrid &grid, char value,
	geometry_msgs::Point A, geometry_msgs::Point B, geometry_msgs::Point C, int mode = COVER_ALL);

void fillPolygon(nav_msgs::OccupancyGrid &grid, char value, vector<geometry_msgs::Point> points, int mode = COVER_ALL);

void fillLine(nav_msgs::OccupancyGrid &grid, char value, geometry_msgs::Point A, geometry_msgs::Point B, int mode = COVER_ALL);

// eval input: -1-1, output: 0-1
// points[0] and points[2] in the inner border, points[1] and points[3] in the outer border
void fillLaneSegment(nav_msgs::OccupancyGrid &grid, vector<geometry_msgs::Point> points, double (*eval)(double l), int mode = COVER_ALL);

//-------------for self defined grid-------------------//
void fillTriangle(planning_matrix::Grid &grid, char value,
	geometry_msgs::Point A, geometry_msgs::Point B, geometry_msgs::Point C, int mode = COVER_ALL);

void fillPolygon(planning_matrix::Grid &grid, char value, vector<geometry_msgs::Point> points, int mode = COVER_ALL);

void fillLine(planning_matrix::Grid &grid, char value, geometry_msgs::Point A, geometry_msgs::Point B, int mode = COVER_ALL);

// eval input: -1-1, output: 0-1
// points[0] and points[2] in the inner border, points[1] and points[3] in the outer border
void fillLaneSegment(planning_matrix::Grid &grid, vector<geometry_msgs::Point> points, double (*eval)(double l), int mode = COVER_ALL);

}

#endif
