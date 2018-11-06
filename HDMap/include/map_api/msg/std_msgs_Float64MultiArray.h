#ifndef STD_MSGS_FLOAT64MULTIARRAY_H
#define STD_MSGS_FLOAT64MULTIARRAY_H

#include "std_msgs_MultiArrayLayout.h"
#include <string>
#include <vector>
using namespace std;

namespace std_msgs {

class Float64MultiArray
{
public:
  Float64MultiArray() {};
  ~Float64MultiArray() {};

  std_msgs::MultiArrayLayout layout;
  vector<double> data;
};
} // namespace std_msgs

#endif
