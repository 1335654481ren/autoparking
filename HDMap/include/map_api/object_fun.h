/********************************************************
*   Copyright (C) 2018 All rights reserved.
*   
*   Filename: object_fun.h
*   Author  : lubing.han
*   Date    : 2018-02-27
*   Describe: object_fun.h
*
********************************************************/
#ifndef OPENDRIVE_OBJECT_FUN_H
#define OPENDRIVE_OBJECT_FUN_H
#include "segment_list.h"
#include "opendrive_object.h"
#ifndef DEROS
#include <autodrive_msgs/opendrive_object_msg.h>
#else
#include "autodrive_msgs_opendrive_object_msg.h"
#endif
using namespace std;

namespace opendrive {

SegmentList toSegmentList(const Object& object);

}

#endif