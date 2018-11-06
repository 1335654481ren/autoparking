#ifndef GEOMETRY_MSGS_ACCEL_H
#define GEOMETRY_MSGS_ACCEL_H

#include "geometry_msgs_Vector3.h"
#include "geometry_msgs_Vector3.h"
#include <string>
#include <vector>
using namespace std;

namespace geometry_msgs {

class Accel
{
public:
  Accel() {};
  ~Accel() {};

  geometry_msgs::Vector3 linear;
  geometry_msgs::Vector3 angular;
};
} // namespace geometry_msgs

#endif
