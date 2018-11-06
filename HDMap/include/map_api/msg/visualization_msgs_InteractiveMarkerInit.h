#ifndef VISUALIZATION_MSGS_INTERACTIVEMARKERINIT_H
#define VISUALIZATION_MSGS_INTERACTIVEMARKERINIT_H

#include "visualization_msgs_InteractiveMarker.h"
#include <string>
#include <vector>
using namespace std;

namespace visualization_msgs {

class InteractiveMarkerInit
{
public:
  InteractiveMarkerInit() {};
  ~InteractiveMarkerInit() {};

  string server_id;
  unsigned long long seq_num;
  vector<visualization_msgs::InteractiveMarker> markers;
};
} // namespace visualization_msgs

#endif
