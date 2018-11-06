#ifndef GEOMETRY_MSGS_POINT_H
#define GEOMETRY_MSGS_POINT_H

#include <string>
#include <vector>
using namespace std;

namespace geometry_msgs {

class Point
{
public:
  Point() {};
  ~Point() {};

  double x;
  double y;
  double z;
};
} // namespace geometry_msgs

#endif
