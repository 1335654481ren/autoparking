#ifndef GEOMETRY_MSGS_TWISTSTAMPED_H
#define GEOMETRY_MSGS_TWISTSTAMPED_H

#include "std_msgs_Header.h"
#include "geometry_msgs_Twist.h"
#include <string>
#include <vector>
using namespace std;

namespace geometry_msgs {

class TwistStamped
{
public:
  TwistStamped() {};
  ~TwistStamped() {};

  std_msgs::Header header;
  geometry_msgs::Twist twist;
};
} // namespace geometry_msgs

#endif
