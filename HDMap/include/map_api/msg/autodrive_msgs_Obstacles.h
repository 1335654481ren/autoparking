#ifndef AUTODRIVE_MSGS_OBSTACLES_H
#define AUTODRIVE_MSGS_OBSTACLES_H

#include "std_msgs_Header.h"
#include "autodrive_msgs_Obstacle.h"
#include "std_msgs_ColorRGBA.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class Obstacles
{
public:
  Obstacles() {};
  ~Obstacles() {};

  std_msgs::Header header;
  vector<autodrive_msgs::Obstacle> obs;
  std_msgs::ColorRGBA color;
};
} // namespace autodrive_msgs

#endif
