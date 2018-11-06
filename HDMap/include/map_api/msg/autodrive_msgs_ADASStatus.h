#ifndef AUTODRIVE_MSGS_ADASSTATUS_H
#define AUTODRIVE_MSGS_ADASSTATUS_H

#include "std_msgs_Header.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class ADASStatus
{
public:
  ADASStatus() {};
  ~ADASStatus() {};

  std_msgs::Header header;
  int adas_adjlane_num;
  double adas_adjlane_func_al;
  double adas_adjlane_func_bl;
  double adas_adjlane_func_cl;
  double adas_adjlane_func_dl;
  double adas_adjlane_func_ar;
  double adas_adjlane_func_br;
  double adas_adjlane_func_cr;
  double adas_adjlane_func_dr;
  double adas_adjlane_bias_l;
  double adas_adjlane_bias_r;
  double adas_lane_center_a;
  double adas_lane_center_b;
  double adas_lane_center_c;
  double adas_lane_center_d;
  double current_bias;
  double adas_v_target;
  double adas_va_target;
  double adas_x_target;
  double adas_y_target;
  double adas_alpha_target;
  double adas_steer_target;
  double adas_steerrate_target;
  double adas_throttle_target;
  double adas_brake_target;
  int ADAS_Exit_Code;
  unsigned char ACC_Status;
  unsigned char AEB_Status;
  unsigned char LKS_Status;
  unsigned char LDW_Status;
};
} // namespace autodrive_msgs

#endif
