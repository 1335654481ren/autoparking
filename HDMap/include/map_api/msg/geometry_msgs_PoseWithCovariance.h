#ifndef GEOMETRY_MSGS_POSEWITHCOVARIANCE_H
#define GEOMETRY_MSGS_POSEWITHCOVARIANCE_H

#include "geometry_msgs_Pose.h"
#include <string>
#include <vector>
using namespace std;

namespace geometry_msgs {

class PoseWithCovariance
{
public:
  PoseWithCovariance() {};
  ~PoseWithCovariance() {};

  geometry_msgs::Pose pose;
  double covariance[36];
};
} // namespace geometry_msgs

#endif
