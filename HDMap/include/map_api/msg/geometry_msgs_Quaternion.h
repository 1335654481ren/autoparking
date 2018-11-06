#ifndef GEOMETRY_MSGS_QUATERNION_H
#define GEOMETRY_MSGS_QUATERNION_H

#include <string>
#include <vector>
using namespace std;

namespace geometry_msgs {

class Quaternion
{
public:
  Quaternion() {};
  ~Quaternion() {};

  double x;
  double y;
  double z;
  double w;
};
} // namespace geometry_msgs

#endif
