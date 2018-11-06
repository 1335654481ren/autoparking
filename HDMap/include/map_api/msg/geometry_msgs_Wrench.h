#ifndef GEOMETRY_MSGS_WRENCH_H
#define GEOMETRY_MSGS_WRENCH_H

#include "geometry_msgs_Vector3.h"
#include "geometry_msgs_Vector3.h"
#include <string>
#include <vector>
using namespace std;

namespace geometry_msgs {

class Wrench
{
public:
  Wrench() {};
  ~Wrench() {};

  geometry_msgs::Vector3 force;
  geometry_msgs::Vector3 torque;
};
} // namespace geometry_msgs

#endif
