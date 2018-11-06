/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename:Spiral.cpp
*   Author  :weiwei.liu
*   Date    :2016-11-14
*   Describe:
*
********************************************************/

#include "Spiral.h"
#include "odrSpiral.h"
#include <math.h>
#include <algorithm>

namespace opendrive {
    
Spiral::Spiral() {
    _start_cuv = 0;
    _end_cuv = 0;
}

Spiral::Spiral(Point& start, double len, double hdg, double start_cuv, double end_cuv) : Geometry(start, len, hdg) {
    _start_cuv = start_cuv;
    _end_cuv = end_cuv;
}

Spiral::~Spiral() {

}

void Spiral::get_points(std::vector<Point>& points, double density) {
    double tmp_x, tmp_y, tmp_theta;
    double cuv_rate;
    bool orientation;
    if ( fabs(_start_cuv) < 1.0e-15 && fabs(_end_cuv) > 1.0e-15 ) {
        orientation = true;
        cuv_rate = (_end_cuv - _start_cuv) / get_len();
    }
    else if ( fabs(_start_cuv) > 1.0e-15 && fabs(_end_cuv) < 1.0e-15 ) {
        orientation = false;
        cuv_rate = -(_start_cuv - _end_cuv) / get_len();
    }
    else {
        return;    
    }
    int num = get_len() / density + 1;
    odrSpiral(get_len(),  cuv_rate, &tmp_x, &tmp_y, &tmp_theta);
    Point end_point(tmp_x, tmp_y, 0.0);
    double end_theta = tmp_theta;
    for (int i = 0; i < num; i++) {
        odrSpiral(i * density, cuv_rate, &tmp_x, &tmp_y, &tmp_theta);
        if (orientation) {
            
            points.push_back(Point(
                // convert to global axis
                get_start().x + tmp_x * cos(get_hdg()) - tmp_y * sin(get_hdg()),
                get_start().y + tmp_x * sin(get_hdg()) + tmp_y * cos(get_hdg()),
                get_start().z,
                i * density,
                theta_unify( get_hdg() + tmp_theta )
            ));
        }
        else {
            // reverse start and end
            double x_to_end = (tmp_x - end_point.x) * cos(end_theta) + (tmp_y - end_point.y) * sin(end_theta);
            double y_to_end = (tmp_y - end_point.y) * cos(end_theta) - (tmp_x - end_point.x) * sin(end_theta);

            points.push_back(Point(
                // convert to global axis
                get_start().x - x_to_end * cos(get_hdg()) + y_to_end * sin(get_hdg()),
                get_start().y - x_to_end * sin(get_hdg()) - y_to_end * cos(get_hdg()),
                get_start().z,
                get_len() - i * density,
                theta_unify( tmp_theta - end_theta + get_hdg() )
            ));
            
        }
    }
    if (!orientation)
        reverse(points.begin(), points.end());

}

Geometry* Spiral::clone() {
    Spiral* spiral = new Spiral();
    *spiral = *this;
    return spiral;
}

}

