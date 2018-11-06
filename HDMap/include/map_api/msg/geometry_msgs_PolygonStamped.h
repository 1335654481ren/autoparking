#ifndef GEOMETRY_MSGS_POLYGONSTAMPED_H
#define GEOMETRY_MSGS_POLYGONSTAMPED_H

#include "std_msgs_Header.h"
#include "geometry_msgs_Polygon.h"
#include <string>
#include <vector>
using namespace std;

namespace geometry_msgs {

class PolygonStamped
{
public:
  PolygonStamped() {};
  ~PolygonStamped() {};

  std_msgs::Header header;
  geometry_msgs::Polygon polygon;
};
} // namespace geometry_msgs

#endif
