#ifndef VISUALIZATION_MSGS_INTERACTIVEMARKERUPDATE_H
#define VISUALIZATION_MSGS_INTERACTIVEMARKERUPDATE_H

#include "visualization_msgs_InteractiveMarker.h"
#include "visualization_msgs_InteractiveMarkerPose.h"
#include <string>
#include <vector>
using namespace std;

namespace visualization_msgs {

class InteractiveMarkerUpdate
{
public:
  InteractiveMarkerUpdate() {};
  ~InteractiveMarkerUpdate() {};

  enum {
    KEEP_ALIVE = 0,
    UPDATE = 1
  };

  string server_id;
  unsigned long long seq_num;
  unsigned char type;
  vector<visualization_msgs::InteractiveMarker> markers;
  vector<visualization_msgs::InteractiveMarkerPose> poses;
  vector<string> erases;
};
} // namespace visualization_msgs

#endif
