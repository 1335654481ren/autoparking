#ifndef GEOMETRY_MSGS_POLYGON_H
#define GEOMETRY_MSGS_POLYGON_H

#include "geometry_msgs_Point32.h"
#include <string>
#include <vector>
using namespace std;

namespace geometry_msgs {

class Polygon
{
public:
  Polygon() {};
  ~Polygon() {};

  vector<geometry_msgs::Point32> points;
};
} // namespace geometry_msgs

#endif
