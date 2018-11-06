/********************************************************
*   Copyright (C) 2018 All rights reserved.
*   
*   Filename: tf_transform_datatypes.cpp
*   Author  : lubing.han
*   Date    : 2018-05-16
*   Describe: 
*
********************************************************/
#include <math.h>
#include "tf_transform_datatypes.h"

namespace tf {

double getYaw(const geometry_msgs::Quaternion& q) {
  double siny = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return atan2(siny, cosy);
}

geometry_msgs::Quaternion createQuaternionMsgFromYaw(double yaw) {
  geometry_msgs::Quaternion q;
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cr = 1.0;
  double sr = 0.0;
  double cp = 1.0;
  double sp = 0.0;

  q.w = cy * cr * cp + sy * sr * sp;
  q.x = cy * sr * cp - sy * cr * sp;
  q.y = cy * cr * sp + sy * sr * cp;
  q.z = sy * cr * cp - cy * sr * sp;
  return q;
}

} // namespace tf