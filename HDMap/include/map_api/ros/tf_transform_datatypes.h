/********************************************************
*   Copyright (C) 2018 All rights reserved.
*   
*   Filename: tf_transform_datatypes.h
*   Author  : lubing.han
*   Date    : 2018-05-16
*   Describe: 
*
********************************************************/
#ifndef ROS
#ifndef TF_TRANSFORM_DATATYPES_H
#define TF_TRANSFORM_DATATYPES_H

#include <math.h>
#include "geometry_msgs_Quaternion.h"

namespace tf {

double getYaw(const geometry_msgs::Quaternion& q);

geometry_msgs::Quaternion createQuaternionMsgFromYaw(double yaw);

} // namespace tf

#endif
#endif