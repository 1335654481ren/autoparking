#ifndef AUTODRIVE_MSGS_POINT_H
#define AUTODRIVE_MSGS_POINT_H

#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class Point
{
public:
  Point() {};
  ~Point() {};

  double x;
  double y;
  double z;
  double s;
  double k;
  double theta;
  int type;
};
} // namespace autodrive_msgs

#endif
