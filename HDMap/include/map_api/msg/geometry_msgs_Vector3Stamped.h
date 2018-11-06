#ifndef GEOMETRY_MSGS_VECTOR3STAMPED_H
#define GEOMETRY_MSGS_VECTOR3STAMPED_H

#include "std_msgs_Header.h"
#include "geometry_msgs_Vector3.h"
#include <string>
#include <vector>
using namespace std;

namespace geometry_msgs {

class Vector3Stamped
{
public:
  Vector3Stamped() {};
  ~Vector3Stamped() {};

  std_msgs::Header header;
  geometry_msgs::Vector3 vector;
};
} // namespace geometry_msgs

#endif
