#ifndef NAV_MSGS_OCCUPANCYGRID_H
#define NAV_MSGS_OCCUPANCYGRID_H

#include "std_msgs_Header.h"
#include "nav_msgs_MapMetaData.h"
#include <string>
#include <vector>
using namespace std;

namespace nav_msgs {

class OccupancyGrid
{
public:
  OccupancyGrid() {};
  ~OccupancyGrid() {};

  std_msgs::Header header;
  nav_msgs::MapMetaData info;
  vector<signed char> data;
};
} // namespace nav_msgs

#endif
