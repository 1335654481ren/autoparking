#ifndef GEOMETRY_MSGS_ACCELWITHCOVARIANCESTAMPED_H
#define GEOMETRY_MSGS_ACCELWITHCOVARIANCESTAMPED_H

#include "std_msgs_Header.h"
#include "geometry_msgs_AccelWithCovariance.h"
#include <string>
#include <vector>
using namespace std;

namespace geometry_msgs {

class AccelWithCovarianceStamped
{
public:
  AccelWithCovarianceStamped() {};
  ~AccelWithCovarianceStamped() {};

  std_msgs::Header header;
  geometry_msgs::AccelWithCovariance accel;
};
} // namespace geometry_msgs

#endif
