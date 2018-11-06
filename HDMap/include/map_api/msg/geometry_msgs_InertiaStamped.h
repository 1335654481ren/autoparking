#ifndef GEOMETRY_MSGS_INERTIASTAMPED_H
#define GEOMETRY_MSGS_INERTIASTAMPED_H

#include "std_msgs_Header.h"
#include "geometry_msgs_Inertia.h"
#include <string>
#include <vector>
using namespace std;

namespace geometry_msgs {

class InertiaStamped
{
public:
  InertiaStamped() {};
  ~InertiaStamped() {};

  std_msgs::Header header;
  geometry_msgs::Inertia inertia;
};
} // namespace geometry_msgs

#endif
