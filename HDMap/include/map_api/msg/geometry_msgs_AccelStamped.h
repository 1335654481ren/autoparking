#ifndef GEOMETRY_MSGS_ACCELSTAMPED_H
#define GEOMETRY_MSGS_ACCELSTAMPED_H

#include "std_msgs_Header.h"
#include "geometry_msgs_Accel.h"
#include <string>
#include <vector>
using namespace std;

namespace geometry_msgs {

class AccelStamped
{
public:
  AccelStamped() {};
  ~AccelStamped() {};

  std_msgs::Header header;
  geometry_msgs::Accel accel;
};
} // namespace geometry_msgs

#endif
