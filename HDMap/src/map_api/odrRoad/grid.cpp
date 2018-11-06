/********************************************************
*   Copyright (C) 2017 All rights reserved.
*
*   Filename: grid.cpp
*   Author  : penghong.lin
*   Date    : Mar 12, 2018
*   Describe:
*
********************************************************/
#include "grid.h"
#include <memory.h>

using namespace planning_matrix;

Grid::Grid(const int grid_width, float resolution) {
	info.grid_width = grid_width;
	info.grid_height = grid_width;
	info.grid_center_x = 0;
	info.grid_center_y = 0;
	info.grid_yaw = 0;
	info.stamp = 0.0;
	info.resolution = resolution;
	data.resize(grid_width*grid_width);
}

Grid::~Grid() {

}

void Grid::set_all(int value) {
	fill(data.begin(), data.end(), value);
}

int Grid::get_grid_width() const {
	return info.grid_width;
}

float Grid::get_resolution() const {
	return info.resolution;
}
