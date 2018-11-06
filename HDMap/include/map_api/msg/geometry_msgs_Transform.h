#ifndef GEOMETRY_MSGS_TRANSFORM_H
#define GEOMETRY_MSGS_TRANSFORM_H

#include "geometry_msgs_Vector3.h"
#include "geometry_msgs_Quaternion.h"
#include <string>
#include <vector>
using namespace std;

namespace geometry_msgs {

class Transform
{
public:
  Transform() {};
  ~Transform() {};

  geometry_msgs::Vector3 translation;
  geometry_msgs::Quaternion rotation;
};
} // namespace geometry_msgs

#endif
