#ifndef VISUALIZATION_MSGS_MARKERARRAY_H
#define VISUALIZATION_MSGS_MARKERARRAY_H

#include "visualization_msgs_Marker.h"
#include <string>
#include <vector>
using namespace std;

namespace visualization_msgs {

class MarkerArray
{
public:
  MarkerArray() {};
  ~MarkerArray() {};

  vector<visualization_msgs::Marker> markers;
};
} // namespace visualization_msgs

#endif
