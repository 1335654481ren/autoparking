#ifndef GEOMETRY_MSGS_POINTSTAMPED_H
#define GEOMETRY_MSGS_POINTSTAMPED_H

#include "std_msgs_Header.h"
#include "geometry_msgs_Point.h"
#include <string>
#include <vector>
using namespace std;

namespace geometry_msgs {

class PointStamped
{
public:
  PointStamped() {};
  ~PointStamped() {};

  std_msgs::Header header;
  geometry_msgs::Point point;
};
} // namespace geometry_msgs

#endif
