/********************************************************
*   Copyright (C) 2018 All rights reserved.
*   
*   Filename: ros_ros.h
*   Author  : lubing.han
*   Date    : 2018-05-16
*   Describe: 
*
********************************************************/
#ifndef ROS
#ifndef ROS_ROS_H
#define ROS_ROS_H

#include <stdio.h>
#include <sys/time.h>

#define ROS_ERROR(format, args...) fprintf(stderr, format, ##args)

namespace ros {

class Time
{
public:
  static double now() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return double(tv.tv_sec) + double(tv.tv_usec) / 1000000.0;
  }
};

} // namespace ros

#endif
#endif