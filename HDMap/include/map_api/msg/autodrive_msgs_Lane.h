#ifndef AUTODRIVE_MSGS_LANE_H
#define AUTODRIVE_MSGS_LANE_H

#include "autodrive_msgs_Point.h"
#include "autodrive_msgs_GlobalLaneId.h"
#include "autodrive_msgs_RoadMark.h"
#include "autodrive_msgs_GlobalLaneId.h"
#include "autodrive_msgs_GlobalLaneId.h"
#include "autodrive_msgs_GlobalLaneId.h"
#include "autodrive_msgs_GlobalLaneId.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class Lane
{
public:
  Lane() {};
  ~Lane() {};

  vector<autodrive_msgs::Point> points;
  autodrive_msgs::GlobalLaneId globalid;
  vector<autodrive_msgs::RoadMark> roadMarks;
  vector<autodrive_msgs::GlobalLaneId> predecessors;
  vector<autodrive_msgs::GlobalLaneId> successors;
  vector<autodrive_msgs::GlobalLaneId> leftNeighbour;
  vector<autodrive_msgs::GlobalLaneId> rightNeighbour;
};
} // namespace autodrive_msgs

#endif
