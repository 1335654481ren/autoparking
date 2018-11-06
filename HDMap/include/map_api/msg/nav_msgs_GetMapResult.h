#ifndef NAV_MSGS_GETMAPRESULT_H
#define NAV_MSGS_GETMAPRESULT_H

#include "nav_msgs_OccupancyGrid.h"
#include <string>
#include <vector>
using namespace std;

namespace nav_msgs {

class GetMapResult
{
public:
  GetMapResult() {};
  ~GetMapResult() {};

  nav_msgs::OccupancyGrid map;
};
} // namespace nav_msgs

#endif
