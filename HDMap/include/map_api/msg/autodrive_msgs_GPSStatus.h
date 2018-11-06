#ifndef AUTODRIVE_MSGS_GPSSTATUS_H
#define AUTODRIVE_MSGS_GPSSTATUS_H

#include "std_msgs_Header.h"
#include "geometry_msgs_Point.h"
#include "geometry_msgs_Point.h"
#include "geometry_msgs_Quaternion.h"
#include "geometry_msgs_Point.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class GPSStatus
{
public:
  GPSStatus() {};
  ~GPSStatus() {};

  std_msgs::Header header;
  float speed;
  geometry_msgs::Point velocity;
  geometry_msgs::Point acc;
  geometry_msgs::Quaternion orientation;
  geometry_msgs::Point position;
};
} // namespace autodrive_msgs

#endif
