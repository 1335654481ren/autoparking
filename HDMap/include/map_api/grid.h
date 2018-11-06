/********************************************************
*   Copyright (C) 2017 All rights reserved.
*
*   Filename: grid.h
*   Author  : penghong.lin
*   Date    : Mar 12, 2018
*   Describe:
*
********************************************************/
#ifndef PLANNER_INCLUDE_STRUCTURE_GRID_H_
#define PLANNER_INCLUDE_STRUCTURE_GRID_H_
#include <vector>
#include <stdint.h>
namespace planning_matrix {

struct GridInfo {
	double stamp;
	float resolution;
	int grid_width;
	int grid_height;
	float grid_center_x;
	float grid_center_y;
	float grid_yaw;
};

class Grid {
public:
	Grid(int grid_width, float resolution);
	~Grid();
	void set_all(int value);
	float get_resolution() const;
	int get_grid_width() const;

	GridInfo info;
	std::vector<int> data;

};

}

#endif /* PLANNER_INCLUDE_STRUCTURE_GRID_H_ */
