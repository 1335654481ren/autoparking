#ifndef AUTODRIVE_MSGS_CARSTATUS_H
#define AUTODRIVE_MSGS_CARSTATUS_H

#include "std_msgs_Header.h"
#include "geometry_msgs_Point.h"
#include "geometry_msgs_Point.h"
#include "geometry_msgs_Point.h"
#include "geometry_msgs_Quaternion.h"
#include "geometry_msgs_Point.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class CarStatus
{
public:
  CarStatus() {};
  ~CarStatus() {};

  std_msgs::Header header;
  float speed;
  geometry_msgs::Point velocity;
  geometry_msgs::Point acc;
  geometry_msgs::Point angular_velocity;
  geometry_msgs::Quaternion orientation;
  geometry_msgs::Point position;
  float steer_angle;
  int gear;
  unsigned char left_light_status;
  unsigned char right_light_status;
  unsigned char brake_light_status;
};
} // namespace autodrive_msgs

#endif
