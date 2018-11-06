#ifndef STD_MSGS_COLORRGBA_H
#define STD_MSGS_COLORRGBA_H

#include <string>
#include <vector>
using namespace std;

namespace std_msgs {

class ColorRGBA
{
public:
  ColorRGBA() {};
  ~ColorRGBA() {};

  float r;
  float g;
  float b;
  float a;
};
} // namespace std_msgs

#endif
