#ifndef STD_MSGS_MULTIARRAYLAYOUT_H
#define STD_MSGS_MULTIARRAYLAYOUT_H

#include "std_msgs_MultiArrayDimension.h"
#include <string>
#include <vector>
using namespace std;

namespace std_msgs {

class MultiArrayLayout
{
public:
  MultiArrayLayout() {};
  ~MultiArrayLayout() {};

  vector<std_msgs::MultiArrayDimension> dim;
  unsigned int data_offset;
};
} // namespace std_msgs

#endif
