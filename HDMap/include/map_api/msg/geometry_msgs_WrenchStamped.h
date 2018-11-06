#ifndef GEOMETRY_MSGS_WRENCHSTAMPED_H
#define GEOMETRY_MSGS_WRENCHSTAMPED_H

#include "std_msgs_Header.h"
#include "geometry_msgs_Wrench.h"
#include <string>
#include <vector>
using namespace std;

namespace geometry_msgs {

class WrenchStamped
{
public:
  WrenchStamped() {};
  ~WrenchStamped() {};

  std_msgs::Header header;
  geometry_msgs::Wrench wrench;
};
} // namespace geometry_msgs

#endif
