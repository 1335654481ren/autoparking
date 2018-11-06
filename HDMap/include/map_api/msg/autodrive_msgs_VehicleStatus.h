#ifndef AUTODRIVE_MSGS_VEHICLESTATUS_H
#define AUTODRIVE_MSGS_VEHICLESTATUS_H

#include "std_msgs_Header.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class VehicleStatus
{
public:
  VehicleStatus() {};
  ~VehicleStatus() {};

  std_msgs::Header header;
  float speed;
  unsigned char wheel_speeds_valid;
  unsigned char wheel_speeds_fl_valid;
  unsigned char wheel_speeds_fr_valid;
  unsigned char wheel_speeds_rl_valid;
  unsigned char wheel_speeds_rr_valid;
  float fl_speed;
  float fr_speed;
  float rl_speed;
  float rr_speed;
  unsigned char wheel_pulses_valid;
  int fl_pulse_direction;
  int fr_pulse_direction;
  int rl_pulse_direction;
  int rr_pulse_direction;
  int fl_pulse;
  int fr_pulse;
  int rl_pulse;
  int rr_pulse;
  unsigned char steer_angle_valid;
  float steer_angle;
  unsigned char gear_valid;
  int gear;
  unsigned char torque_valid;
  float torque;
  unsigned char yaw_rate_valid;
  float yaw_rate;
  float yaw_rate_offset;
  unsigned char left_light_status;
  unsigned char right_light_status;
  unsigned char brake_light_status;
};
} // namespace autodrive_msgs

#endif
