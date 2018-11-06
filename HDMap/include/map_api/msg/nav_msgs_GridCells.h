#ifndef NAV_MSGS_GRIDCELLS_H
#define NAV_MSGS_GRIDCELLS_H

#include "std_msgs_Header.h"
#include "geometry_msgs_Point.h"
#include <string>
#include <vector>
using namespace std;

namespace nav_msgs {

class GridCells
{
public:
  GridCells() {};
  ~GridCells() {};

  std_msgs::Header header;
  float cell_width;
  float cell_height;
  vector<geometry_msgs::Point> cells;
};
} // namespace nav_msgs

#endif
