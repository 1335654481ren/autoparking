#ifndef GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED_H
#define GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED_H

#include "std_msgs_Header.h"
#include "geometry_msgs_PoseWithCovariance.h"
#include <string>
#include <vector>
using namespace std;

namespace geometry_msgs {

class PoseWithCovarianceStamped
{
public:
  PoseWithCovarianceStamped() {};
  ~PoseWithCovarianceStamped() {};

  std_msgs::Header header;
  geometry_msgs::PoseWithCovariance pose;
};
} // namespace geometry_msgs

#endif
