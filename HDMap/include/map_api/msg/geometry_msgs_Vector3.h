#ifndef GEOMETRY_MSGS_VECTOR3_H
#define GEOMETRY_MSGS_VECTOR3_H

#include <string>
#include <vector>
using namespace std;

namespace geometry_msgs {

class Vector3
{
public:
  Vector3() {};
  ~Vector3() {};

  double x;
  double y;
  double z;
};
} // namespace geometry_msgs

#endif
