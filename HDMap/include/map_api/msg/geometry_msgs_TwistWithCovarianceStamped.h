#ifndef GEOMETRY_MSGS_TWISTWITHCOVARIANCESTAMPED_H
#define GEOMETRY_MSGS_TWISTWITHCOVARIANCESTAMPED_H

#include "std_msgs_Header.h"
#include "geometry_msgs_TwistWithCovariance.h"
#include <string>
#include <vector>
using namespace std;

namespace geometry_msgs {

class TwistWithCovarianceStamped
{
public:
  TwistWithCovarianceStamped() {};
  ~TwistWithCovarianceStamped() {};

  std_msgs::Header header;
  geometry_msgs::TwistWithCovariance twist;
};
} // namespace geometry_msgs

#endif
