#ifndef AUTODRIVE_MSGS_SEGMENTLIST_H
#define AUTODRIVE_MSGS_SEGMENTLIST_H

#include "autodrive_msgs_opendrive_object_shape_point_msg.h"
#include "autodrive_msgs_opendrive_object_shape_point_msg.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class SegmentList
{
public:
  SegmentList() {};
  ~SegmentList() {};

  int id;
  string type;
  string subtype;
  vector<autodrive_msgs::opendrive_object_shape_point_msg> starts;
  vector<autodrive_msgs::opendrive_object_shape_point_msg> ends;
};
} // namespace autodrive_msgs

#endif
