#ifndef GEOMETRY_MSGS_QUATERNIONSTAMPED_H
#define GEOMETRY_MSGS_QUATERNIONSTAMPED_H

#include "std_msgs_Header.h"
#include "geometry_msgs_Quaternion.h"
#include <string>
#include <vector>
using namespace std;

namespace geometry_msgs {

class QuaternionStamped
{
public:
  QuaternionStamped() {};
  ~QuaternionStamped() {};

  std_msgs::Header header;
  geometry_msgs::Quaternion quaternion;
};
} // namespace geometry_msgs

#endif
