#ifndef STD_MSGS_MULTIARRAYDIMENSION_H
#define STD_MSGS_MULTIARRAYDIMENSION_H

#include <string>
#include <vector>
using namespace std;

namespace std_msgs {

class MultiArrayDimension
{
public:
  MultiArrayDimension() {};
  ~MultiArrayDimension() {};

  string label;
  unsigned int size;
  unsigned int stride;
};
} // namespace std_msgs

#endif
