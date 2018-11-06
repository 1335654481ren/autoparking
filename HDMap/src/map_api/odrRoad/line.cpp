#include <iostream>
#include <math.h>
#include "line.h"
using namespace std;

namespace opendrive
{
Line::Line(Point &Start,double Len,double Hdg)
{
    set_start(Start);
    set_len(Len);
    set_hdg(Hdg);
};

void Line::get_points(std::vector<Point>& points,double density)
{    
    int n=floor(get_len()/density) + 1;
    points.clear();
    for(int i=0;i<n;i++)
    {
        double x=get_start().x+cos(get_hdg())*i*density;
        double y=get_start().y+sin(get_hdg())*i*density;
        double z=get_start().z;
        Point p(x, y, z, i * density, get_hdg());
        points.push_back(p);
    }

        
}

Geometry* Line::clone() {
    Line* line = new Line();
    *line = *this;
    return line;
}

}


    
