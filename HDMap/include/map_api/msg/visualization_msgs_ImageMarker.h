#ifndef VISUALIZATION_MSGS_IMAGEMARKER_H
#define VISUALIZATION_MSGS_IMAGEMARKER_H

#include "std_msgs_Header.h"
#include "geometry_msgs_Point.h"
#include "std_msgs_ColorRGBA.h"
#include "std_msgs_ColorRGBA.h"
#include "geometry_msgs_Point.h"
#include "std_msgs_ColorRGBA.h"
#include <string>
#include <vector>
using namespace std;

namespace visualization_msgs {

class ImageMarker
{
public:
  ImageMarker() {};
  ~ImageMarker() {};

  enum {
    CIRCLE = 0,
    LINE_STRIP = 1,
    LINE_LIST = 2,
    POLYGON = 3,
    POINTS = 4,
    ADD = 0,
    REMOVE = 1
  };

  std_msgs::Header header;
  string ns;
  int id;
  int type;
  int action;
  geometry_msgs::Point position;
  float scale;
  std_msgs::ColorRGBA outline_color;
  unsigned char filled;
  std_msgs::ColorRGBA fill_color;
  double lifetime;
  vector<geometry_msgs::Point> points;
  vector<std_msgs::ColorRGBA> outline_colors;
};
} // namespace visualization_msgs

#endif
