/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: map_api_parameters.h
*   Author  : lubing.han
*   Date    : 2016-11-16
*   Describe:
*
********************************************************/
#ifndef MAP_API_PARAMETERS_H
#define MAP_API_PARAMETERS_H

namespace opendrive
{

#define XY_Precision 0.01
#define Z_Precision 0.005
#define Max_Point_Distance 50.0
// -1 Left, 1 Right
#define Drive_Direction 1
#define Junction_Extend_Length 30.0
#define Front_Extend_Length 400.0
#define Front_Min_Extend_Length 200.0
#define Back_Extend_Length 100.0
#define GRID_RESOLUTION 0.2
#define GRID_WIDTH 500.0
#define GRID_HEIGHT 500.0
}

#endif
