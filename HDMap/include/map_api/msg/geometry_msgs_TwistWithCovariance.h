#ifndef GEOMETRY_MSGS_TWISTWITHCOVARIANCE_H
#define GEOMETRY_MSGS_TWISTWITHCOVARIANCE_H

#include "geometry_msgs_Twist.h"
#include <string>
#include <vector>
using namespace std;

namespace geometry_msgs {

class TwistWithCovariance
{
public:
  TwistWithCovariance() {};
  ~TwistWithCovariance() {};

  geometry_msgs::Twist twist;
  double covariance[36];
};
} // namespace geometry_msgs

#endif
