#ifndef NAV_MSGS_ODOMETRY_H
#define NAV_MSGS_ODOMETRY_H

#include "std_msgs_Header.h"
#include "geometry_msgs_PoseWithCovariance.h"
#include "geometry_msgs_TwistWithCovariance.h"
#include <string>
#include <vector>
using namespace std;

namespace nav_msgs {

class Odometry
{
public:
  Odometry() {};
  ~Odometry() {};

  std_msgs::Header header;
  string child_frame_id;
  geometry_msgs::PoseWithCovariance pose;
  geometry_msgs::TwistWithCovariance twist;
};
} // namespace nav_msgs

#endif
