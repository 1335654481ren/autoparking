#ifndef GEOMETRY_MSGS_POSE2D_H
#define GEOMETRY_MSGS_POSE2D_H

#include <string>
#include <vector>
using namespace std;

namespace geometry_msgs {

class Pose2D
{
public:
  Pose2D() {};
  ~Pose2D() {};

  double x;
  double y;
  double theta;
};
} // namespace geometry_msgs

#endif
