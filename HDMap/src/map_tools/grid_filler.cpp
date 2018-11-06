/********************************************************
*   Copyright (C) 2016 All rights reserved.
*
*   Filename: grid_filler.cpp
*   Author  : lubing.han
*   Date    : 2017-07-05
*   Describe: 
*
********************************************************/
#include "grid_filler.h"

#ifndef DEROS
#include <tf/transform_datatypes.h>
#else
#include "tf_transform_datatypes.h"
#endif
#include <math.h>
#include <iostream>
using namespace std;

namespace opendrive {

void fillTriangle(vector<int8_t> &grid, int width, int height, double step, char value,
	geometry_msgs::Point &A, geometry_msgs::Point &B, geometry_msgs::Point &C, int mode);

void fillLine(vector<int8_t> &grid, int width, int height, double step, char value,
	geometry_msgs::Point &A, geometry_msgs::Point &B, int mode);

void transXY(const geometry_msgs::Pose &pose, geometry_msgs::Point &point);


void fillTriangle(nav_msgs::OccupancyGrid &grid, char value,
	geometry_msgs::Point A, geometry_msgs::Point B, geometry_msgs::Point C, int mode) {
	transXY(grid.info.origin, A);
	transXY(grid.info.origin, B);
	transXY(grid.info.origin, C);
	fillTriangle(grid.data, grid.info.width, grid.info.height, grid.info.resolution, value, A, B, C, mode);
}

void fillPolygon(nav_msgs::OccupancyGrid &grid, char value, vector<geometry_msgs::Point> points, int mode) {
	if (points.size() == 2) {
		fillLine(grid, value, points[0], points[1], mode);
	}
	else {
		for (int i = 1; i + 1 < points.size(); ++i)
			fillTriangle(grid, value, points[0], points[i], points[i + 1], mode);
	}
}

void fillLine(nav_msgs::OccupancyGrid &grid, char value, geometry_msgs::Point A, geometry_msgs::Point B, int mode) {
	transXY(grid.info.origin, A);
	transXY(grid.info.origin, B);
	fillLine(grid.data, grid.info.width, grid.info.height, grid.info.resolution, value, A, B, mode);
}

void fillLaneSegment(nav_msgs::OccupancyGrid &grid, vector<geometry_msgs::Point> points, double (*eval)(double l), int mode) {
	transXY(grid.info.origin, points[0]), transXY(grid.info.origin, points[2]);
	transXY(grid.info.origin, points[1]), transXY(grid.info.origin, points[3]);
	double width = grid.info.width * grid.info.resolution;
	double height = grid.info.height * grid.info.resolution;
	if (points[0].x < 0 && points[1].x < 0 && points[2].x < 0 && points[3].x < 0) return;
	if (points[0].y < 0 && points[1].y < 0 && points[2].y < 0 && points[3].y < 0) return;
	if (points[0].x > width && points[1].x > width && points[2].x > width && points[3].x > width) return;
	if (points[0].y > height && points[1].y > height && points[2].y > height && points[3].y > height) return;
	geometry_msgs::Point p01, p23;
	p01.x = points[1].x - points[0].x, p01.y = points[1].y - points[0].y;
	p23.x = points[3].x - points[2].x, p23.y = points[3].y - points[2].y;
	int n01 = int(sqrt(p01.x * p01.x + p01.y * p01.y) / grid.info.resolution) + 2;
	int n23 = int(sqrt(p23.x * p23.x + p23.y * p23.y) / grid.info.resolution) + 2;
	int n = n01 > n23 ? n01 : n23;
	for (int i = 0; i < n; ++ i) {
		double proportion = double(i) / (n - 1);
		geometry_msgs::Point A = points[0], B = points[2];
		A.x += proportion * p01.x, A.y += proportion * p01.y;
		B.x += proportion * p23.x, B.y += proportion * p23.y;
		int value = int(eval(2 * proportion - 1) * 100 + 0.5);
		value = value > 100 ? 100 : value;
		value = value < 0 ? 0 : value;
		fillLine(grid.data, grid.info.width, grid.info.height, grid.info.resolution, value, A, B, mode);
	}
}

void fillTriangle(vector<int8_t> &grid, int width, int height, double step, char value,
	geometry_msgs::Point &A, geometry_msgs::Point &B, geometry_msgs::Point &C, int mode) {
	double map_width = width * step;
	double map_height = height * step;
	if (A.x < 0 && B.x < 0 && C.x < 0) return;
	if (A.y < 0 && B.y < 0 && C.y < 0) return;
	if (A.x > map_width && B.x > map_width && C.x > map_width) return;
	if (A.y > map_height && B.y > map_height && C.y > map_height) return;
	double xa, ya, xb, yb, xc, yc;
	if      (A.y >= B.y && B.y >= C.y) xa = A.x, ya = A.y, xb = B.x, yb = B.y, xc = C.x, yc = C.y;
	else if (A.y >= C.y && C.y >= B.y) xa = A.x, ya = A.y, xb = C.x, yb = C.y, xc = B.x, yc = B.y;
	else if (B.y >= A.y && A.y >= C.y) xa = B.x, ya = B.y, xb = A.x, yb = A.y, xc = C.x, yc = C.y;
	else if (B.y >= C.y && C.y >= A.y) xa = B.x, ya = B.y, xb = C.x, yb = C.y, xc = A.x, yc = A.y;
	else if (C.y >= A.y && A.y >= B.y) xa = C.x, ya = C.y, xb = A.x, yb = A.y, xc = B.x, yc = B.y;
	else if (C.y >= B.y && B.y >= A.y) xa = C.x, ya = C.y, xb = B.x, yb = B.y, xc = A.x, yc = A.y;

	// int kymax = int((ya - 0.5 * step) / step);
	int kymax = int((ya) / step);
	kymax = kymax >= height ? height - 1 : kymax;
	// int kymin = int((yc + 0.5 * step) / step);
	int kymin = int((yc) / step);
	kymin = kymin < 0 ? 0 : kymin;
	double yabi = ya - yb < 1e-5 ? 1e5 : 1 / (ya - yb);
	double ybci = yb - yc < 1e-5 ? 1e5 : 1 / (yb - yc);
	double yaci = ya - yc < 1e-5 ? 1e5 : 1 / (ya - yc);

	for (int k = kymax; k >= kymin; --k) {
		// double yk = (k + 0.5) * step;
		double yk, ykp, ykb;
		yk = k * step; ykp = (k + 1) * step;
		if (yk < yc) yk = yc; if (ykp > ya) ykp = ya;
		if (k * step > yb || (k + 1) * step < yb) ykb = yk;
		else ykb = yb;

		double xkbb, xkbt, xkcb, xkct;
		if (yk >= yb) xkbb = (yk - yb) * yabi * (xa - xb) + xb;
		else xkbb = (yk - yc) * ybci * (xb - xc) + xc;
		xkcb = (yk - yc) * yaci * (xa - xc) + xc;
		if (ykp >= yb) xkbt = (ykp - yb) * yabi * (xa - xb) + xb;
		else xkbt = (ykp - yc) * ybci * (xb - xc) + xc;
		xkct = (ykp - yc) * yaci * (xa - xc) + xc;
		double xkb, xkc;
		xkc = (xkbb > xkcb) ^ (xkcb > xkct) ? xkcb : xkct;
		xkb = (xkbb < xkcb) ^ (xkbb > xkbt) ? xkbb : xkbt;
		if (ykb != yk) xkb = (xkbb < xkcb) ^ (xkb > xb) ? xkb : xb;
		// xkb = xkbb; xkc = xkcb;

		double xtemp;
		if (xkc < xkb) xtemp = xkc, xkc = xkb, xkb = xtemp;
		// int kxmax = int((xkc - 0.5 * step) / step);
		int kxmax = int((xkc) / step) + 1;
		kxmax = kxmax >= width ? width - 1 : kxmax;
		// int kxmin = int((xkb + 0.5 * step) / step);
		int kxmin = int((xkb) / step) - 1;
		kxmin = kxmin < 0 ? 0 : kxmin;
		for (int i = width * k + kxmin; i <= width * k + kxmax; ++i) {
			if (mode == COVER_ALL) grid[i] = value;
			else if (mode == COVER_IF_LARGER) grid[i] = value > grid[i] ? value : grid[i];
			else if (mode == COVER_IF_SMALLER) grid[i] = value < grid[i] ? value : grid[i];
			else if (mode == COVER_IF_NONNEG) grid[i] = grid[i] >= 0 ? value : grid[i];
			else if (mode == COVER_IF_NOT_42) grid[i] = grid[i] != 42 ? value : grid[i];
			else if (mode == COVER_IF_42) grid[i] = grid[i] == 42 ? value : grid[i];
		}
	}
}

void fillLine(vector<int8_t> &grid, int width, int height, double step, char value,
	geometry_msgs::Point &A, geometry_msgs::Point &B, int mode) {
	double xa, ya, xb, yb;
	if (A.y >= B.y) xa = A.x, ya = A.y, xb = B.x, yb = B.y;
	else xa = B.x, ya = B.y, xb = A.x, yb = A.y;

	int kymax = int(ya / step) + 1;
	kymax = kymax >= height ? height - 1 : kymax;
	int kymin = int(yb / step);
	kymin = kymin < 0 ? 0 : kymin;
	double yabi = ya - yb < 1e-5 ? 1e5 : 1 / (ya - yb);

	for (int k = kymax; k >= kymin; --k) {
		double ykup = (k + 1) * step;
		ykup = ykup > ya ? ya : ykup;
		double ykdown = k * step;
		ykdown = ykdown < yb ? yb : ykdown;
		if (ykdown > ykup) continue;
		double xkl, xkr;
		xkl = xa - (ya - ykup) * yabi * (xa - xb);
		xkr = (ykdown - yb) * yabi * (xa - xb) + xb;
		double xtemp;
		if (xkr < xkl) xtemp = xkl, xkl = xkr, xkr = xtemp;
		int kxmax = int(xkr / step) + 1;
		kxmax = kxmax >= width ? width - 1 : kxmax;
		int kxmin = int(xkl / step);
		kxmin = kxmin < 0 ? 0 : kxmin;
		for (int i = width * k + kxmin; i <= width * k + kxmax; ++i) {
			if (mode == COVER_ALL) grid[i] = value;
			else if (mode == COVER_IF_LARGER) grid[i] = value > grid[i] ? value : grid[i];
			else if (mode == COVER_IF_SMALLER) grid[i] = value < grid[i] ? value : grid[i];
			else if (mode == COVER_IF_NONNEG) grid[i] = grid[i] >= 0 ? value : grid[i];
			else if (mode == COVER_IF_NOT_42) grid[i] = grid[i] != 42 ? value : grid[i];
			else if (mode == COVER_IF_42) grid[i] = grid[i] == 42 ? value : grid[i];
		}
	}
}

void transXY(const geometry_msgs::Pose &pose, geometry_msgs::Point &point) {
	double dx = point.x - pose.position.x;
	double dy = point.y - pose.position.y;
	double yaw = tf::getYaw(pose.orientation);
	point.x = dx * cos(yaw) + dy * sin(yaw);
	point.y = dy * cos(yaw) - dx * sin(yaw);
}

//---------for self defined grid-----------------//
void fillTriangle(vector<int> &grid, int width, int height, double step, char value,
	geometry_msgs::Point &A, geometry_msgs::Point &B, geometry_msgs::Point &C, int mode);
void fillLine(vector<int> &grid, int width, int height, double step, char value,
	geometry_msgs::Point &A, geometry_msgs::Point &B, int mode);
void transXY(const planning_matrix::GridInfo &info, geometry_msgs::Point &point);

void fillTriangle(planning_matrix::Grid &grid, char value,
	geometry_msgs::Point A, geometry_msgs::Point B, geometry_msgs::Point C, int mode) {
	transXY(grid.info, A);
	transXY(grid.info, B);
	transXY(grid.info, C);
	fillTriangle(grid.data, grid.info.grid_width, grid.info.grid_height, grid.info.resolution, value, A, B, C, mode);
}

void fillPolygon(planning_matrix::Grid &grid, char value, vector<geometry_msgs::Point> points, int mode) {
	if (points.size() == 2) {
		fillLine(grid, value, points[0], points[1], mode);
	}
	else {
		for (int i = 1; i + 1 < points.size(); ++i)
			fillTriangle(grid, value, points[0], points[i], points[i + 1], mode);
	}
}

void fillLine(planning_matrix::Grid &grid, char value, geometry_msgs::Point A, geometry_msgs::Point B, int mode) {
	transXY(grid.info, A);
	transXY(grid.info, B);
	fillLine(grid.data, grid.info.grid_width, grid.info.grid_height, grid.info.resolution, value, A, B, mode);
}

void fillLaneSegment(planning_matrix::Grid &grid, vector<geometry_msgs::Point> points, double (*eval)(double l), int mode) {
	transXY(grid.info, points[0]), transXY(grid.info, points[2]);
	transXY(grid.info, points[1]), transXY(grid.info, points[3]);
	double width = grid.info.grid_width * grid.info.resolution;
	double height = grid.info.grid_height * grid.info.resolution;
	if (points[0].x < 0 && points[1].x < 0 && points[2].x < 0 && points[3].x < 0) return;
	if (points[0].y < 0 && points[1].y < 0 && points[2].y < 0 && points[3].y < 0) return;
	if (points[0].x > width && points[1].x > width && points[2].x > width && points[3].x > width) return;
	if (points[0].y > height && points[1].y > height && points[2].y > height && points[3].y > height) return;
	geometry_msgs::Point p01, p23;
	p01.x = points[1].x - points[0].x, p01.y = points[1].y - points[0].y;
	p23.x = points[3].x - points[2].x, p23.y = points[3].y - points[2].y;
	int n01 = int(sqrt(p01.x * p01.x + p01.y * p01.y) / grid.info.resolution) + 2;
	int n23 = int(sqrt(p23.x * p23.x + p23.y * p23.y) / grid.info.resolution) + 2;
	int n = n01 > n23 ? n01 : n23;
	for (int i = 0; i < n; ++ i) {
		double proportion = double(i) / (n - 1);
		geometry_msgs::Point A = points[0], B = points[2];
		A.x += proportion * p01.x, A.y += proportion * p01.y;
		B.x += proportion * p23.x, B.y += proportion * p23.y;
		int value = int(eval(2 * proportion - 1) * 100 + 0.5);
		value = value > 100 ? 100 : value;
		value = value < 0 ? 0 : value;
		fillLine(grid.data, grid.info.grid_width, grid.info.grid_height, grid.info.resolution, value, A, B, mode);
	}
}

void fillTriangle(vector<int> &grid, int width, int height, double step, char value,
	geometry_msgs::Point &A, geometry_msgs::Point &B, geometry_msgs::Point &C, int mode) {
	double map_width = width * step;
	double map_height = height * step;
	if (A.x < 0 && B.x < 0 && C.x < 0) return;
	if (A.y < 0 && B.y < 0 && C.y < 0) return;
	if (A.x > map_width && B.x > map_width && C.x > map_width) return;
	if (A.y > map_height && B.y > map_height && C.y > map_height) return;
	double xa, ya, xb, yb, xc, yc;
	if      (A.y >= B.y && B.y >= C.y) xa = A.x, ya = A.y, xb = B.x, yb = B.y, xc = C.x, yc = C.y;
	else if (A.y >= C.y && C.y >= B.y) xa = A.x, ya = A.y, xb = C.x, yb = C.y, xc = B.x, yc = B.y;
	else if (B.y >= A.y && A.y >= C.y) xa = B.x, ya = B.y, xb = A.x, yb = A.y, xc = C.x, yc = C.y;
	else if (B.y >= C.y && C.y >= A.y) xa = B.x, ya = B.y, xb = C.x, yb = C.y, xc = A.x, yc = A.y;
	else if (C.y >= A.y && A.y >= B.y) xa = C.x, ya = C.y, xb = A.x, yb = A.y, xc = B.x, yc = B.y;
	else if (C.y >= B.y && B.y >= A.y) xa = C.x, ya = C.y, xb = B.x, yb = B.y, xc = A.x, yc = A.y;

	// int kymax = int((ya - 0.5 * step) / step);
	int kymax = int((ya) / step);
	kymax = kymax >= height ? height - 1 : kymax;
	// int kymin = int((yc + 0.5 * step) / step);
	int kymin = int((yc) / step);
	kymin = kymin < 0 ? 0 : kymin;
	double yabi = ya - yb < 1e-5 ? 1e5 : 1 / (ya - yb);
	double ybci = yb - yc < 1e-5 ? 1e5 : 1 / (yb - yc);
	double yaci = ya - yc < 1e-5 ? 1e5 : 1 / (ya - yc);

	for (int k = kymax; k >= kymin; --k) {
		// double yk = (k + 0.5) * step;
		double yk, ykp, ykb;
		yk = k * step; ykp = (k + 1) * step;
		if (yk < yc) yk = yc; if (ykp > ya) ykp = ya;
		if (k * step > yb || (k + 1) * step < yb) ykb = yk;
		else ykb = yb;

		double xkbb, xkbt, xkcb, xkct;
		if (yk >= yb) xkbb = (yk - yb) * yabi * (xa - xb) + xb;
		else xkbb = (yk - yc) * ybci * (xb - xc) + xc;
		xkcb = (yk - yc) * yaci * (xa - xc) + xc;
		if (ykp >= yb) xkbt = (ykp - yb) * yabi * (xa - xb) + xb;
		else xkbt = (ykp - yc) * ybci * (xb - xc) + xc;
		xkct = (ykp - yc) * yaci * (xa - xc) + xc;
		double xkb, xkc;
		xkc = (xkbb > xkcb) ^ (xkcb > xkct) ? xkcb : xkct;
		xkb = (xkbb < xkcb) ^ (xkbb > xkbt) ? xkbb : xkbt;
		if (ykb != yk) xkb = (xkbb < xkcb) ^ (xkb > xb) ? xkb : xb;
		// xkb = xkbb; xkc = xkcb;

		double xtemp;
		if (xkc < xkb) xtemp = xkc, xkc = xkb, xkb = xtemp;
		// int kxmax = int((xkc - 0.5 * step) / step);
		int kxmax = int((xkc) / step) + 1;
		kxmax = kxmax >= width ? width - 1 : kxmax;
		// int kxmin = int((xkb + 0.5 * step) / step);
		int kxmin = int((xkb) / step) - 1;
		kxmin = kxmin < 0 ? 0 : kxmin;
		for (int i = width * k + kxmin; i <= width * k + kxmax; ++i) {
			if (mode == COVER_ALL) grid[i] = value;
			else if (mode == COVER_IF_LARGER) grid[i] = value > grid[i] ? value : grid[i];
			else if (mode == COVER_IF_SMALLER) grid[i] = value < grid[i] ? value : grid[i];
			else if (mode == COVER_IF_NONNEG) grid[i] = grid[i] >= 0 ? value : grid[i];
			else if (mode == COVER_IF_NOT_42) grid[i] = grid[i] != 42 ? value : grid[i];
			else if (mode == COVER_IF_42) grid[i] = grid[i] == 42 ? value : grid[i];
		}
	}
}

void fillLine(vector<int> &grid, int width, int height, double step, char value,
	geometry_msgs::Point &A, geometry_msgs::Point &B, int mode) {
	double xa, ya, xb, yb;
	if (A.y >= B.y) xa = A.x, ya = A.y, xb = B.x, yb = B.y;
	else xa = B.x, ya = B.y, xb = A.x, yb = A.y;

	int kymax = int(ya / step) + 1;
	kymax = kymax >= height ? height - 1 : kymax;
	int kymin = int(yb / step);
	kymin = kymin < 0 ? 0 : kymin;
	double yabi = ya - yb < 1e-5 ? 1e5 : 1 / (ya - yb);

	for (int k = kymax; k >= kymin; --k) {
		double ykup = (k + 1) * step;
		ykup = ykup > ya ? ya : ykup;
		double ykdown = k * step;
		ykdown = ykdown < yb ? yb : ykdown;
		if (ykdown > ykup) continue;
		double xkl, xkr;
		xkl = xa - (ya - ykup) * yabi * (xa - xb);
		xkr = (ykdown - yb) * yabi * (xa - xb) + xb;
		double xtemp;
		if (xkr < xkl) xtemp = xkl, xkl = xkr, xkr = xtemp;
		int kxmax = int(xkr / step) + 1;
		kxmax = kxmax >= width ? width - 1 : kxmax;
		int kxmin = int(xkl / step);
		kxmin = kxmin < 0 ? 0 : kxmin;
		for (int i = width * k + kxmin; i <= width * k + kxmax; ++i) {
			if (mode == COVER_ALL) grid[i] = value;
			else if (mode == COVER_IF_LARGER) grid[i] = value > grid[i] ? value : grid[i];
			else if (mode == COVER_IF_SMALLER) grid[i] = value < grid[i] ? value : grid[i];
			else if (mode == COVER_IF_NONNEG) grid[i] = grid[i] >= 0 ? value : grid[i];
			else if (mode == COVER_IF_NOT_42) grid[i] = grid[i] != 42 ? value : grid[i];
			else if (mode == COVER_IF_42) grid[i] = grid[i] == 42 ? value : grid[i];
		}
	}
}

void transXY(const planning_matrix::GridInfo &info, geometry_msgs::Point &point) {
	double dx = point.x - info.grid_center_x;
	double dy = point.y - info.grid_center_y;
	double yaw = info.grid_yaw;
	point.x = dx * cos(yaw) + dy * sin(yaw);
	point.y = dy * cos(yaw) - dx * sin(yaw);
}
//---------for self defined grid-----------------//


double simplePiecewiseLinear(double d) {
	return fabs(d) / 2;
}


}
