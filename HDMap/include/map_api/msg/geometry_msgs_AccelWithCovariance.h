#ifndef GEOMETRY_MSGS_ACCELWITHCOVARIANCE_H
#define GEOMETRY_MSGS_ACCELWITHCOVARIANCE_H

#include "geometry_msgs_Accel.h"
#include <string>
#include <vector>
using namespace std;

namespace geometry_msgs {

class AccelWithCovariance
{
public:
  AccelWithCovariance() {};
  ~AccelWithCovariance() {};

  geometry_msgs::Accel accel;
  double covariance[36];
};
} // namespace geometry_msgs

#endif
