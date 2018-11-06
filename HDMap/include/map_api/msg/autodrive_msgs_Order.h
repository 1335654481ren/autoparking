#ifndef AUTODRIVE_MSGS_ORDER_H
#define AUTODRIVE_MSGS_ORDER_H

#include "std_msgs_Header.h"
#include "geometry_msgs_Point.h"
#include "geometry_msgs_Point.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class Order
{
public:
  Order() {};
  ~Order() {};

  std_msgs::Header header;
  int mode;
  unsigned char start;
  geometry_msgs::Point pose;
  geometry_msgs::Point goal;
};
} // namespace autodrive_msgs

#endif
