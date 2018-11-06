#ifndef GEOMETRY_MSGS_INERTIA_H
#define GEOMETRY_MSGS_INERTIA_H

#include "geometry_msgs_Vector3.h"
#include <string>
#include <vector>
using namespace std;

namespace geometry_msgs {

class Inertia
{
public:
  Inertia() {};
  ~Inertia() {};

  double m;
  geometry_msgs::Vector3 com;
  double ixx;
  double ixy;
  double ixz;
  double iyy;
  double iyz;
  double izz;
};
} // namespace geometry_msgs

#endif
