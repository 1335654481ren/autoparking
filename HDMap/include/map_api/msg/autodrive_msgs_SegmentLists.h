#ifndef AUTODRIVE_MSGS_SEGMENTLISTS_H
#define AUTODRIVE_MSGS_SEGMENTLISTS_H

#include "std_msgs_Header.h"
#include "autodrive_msgs_SegmentList.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class SegmentLists
{
public:
  SegmentLists() {};
  ~SegmentLists() {};

  std_msgs::Header header;
  vector<autodrive_msgs::SegmentList> segmentLists;
};
} // namespace autodrive_msgs

#endif
