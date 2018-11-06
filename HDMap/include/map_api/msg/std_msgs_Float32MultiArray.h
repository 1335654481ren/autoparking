#ifndef STD_MSGS_FLOAT32MULTIARRAY_H
#define STD_MSGS_FLOAT32MULTIARRAY_H

#include "std_msgs_MultiArrayLayout.h"
#include <string>
#include <vector>
using namespace std;

namespace std_msgs {

class Float32MultiArray
{
public:
  Float32MultiArray() {};
  ~Float32MultiArray() {};

  std_msgs::MultiArrayLayout layout;
  vector<float> data;
};
} // namespace std_msgs

#endif
