#ifndef NAV_MSGS_PATH_H
#define NAV_MSGS_PATH_H

#include "std_msgs_Header.h"
#include "geometry_msgs_PoseStamped.h"
#include <string>
#include <vector>
using namespace std;

namespace nav_msgs {

class Path
{
public:
  Path() {};
  ~Path() {};

  std_msgs::Header header;
  vector<geometry_msgs::PoseStamped> poses;
};
} // namespace nav_msgs

#endif
