#ifndef AUTODRIVE_MSGS_CONTROL_H
#define AUTODRIVE_MSGS_CONTROL_H

#include "std_msgs_Header.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class Control
{
public:
  Control() {};
  ~Control() {};

  std_msgs::Header header;
  double throttle;
  double steer_target;
  double steer_rate;
  double brake;
  unsigned char turn_left_light;
  unsigned char turn_right_light;
  unsigned char brake_light;
  unsigned char autodrive_mode;
  int gear;
  unsigned char start_request;
  unsigned char stop_request;
  unsigned char downhill_signal;
  unsigned char uphill_signal;
  unsigned char emergency_signal;
  double speed;
  double steer_ref;
};
} // namespace autodrive_msgs

#endif
