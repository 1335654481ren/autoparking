#ifndef GEOMETRY_MSGS_POINT32_H
#define GEOMETRY_MSGS_POINT32_H

#include <string>
#include <vector>
using namespace std;

namespace geometry_msgs {

class Point32
{
public:
  Point32() {};
  ~Point32() {};

  float x;
  float y;
  float z;
};
} // namespace geometry_msgs

#endif
