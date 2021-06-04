#ifndef CALC_DIFF_TIME_H
#define CALC_DIFF_TIME_H

#include <stdlib.h>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace calculators {

  class DiffTimeClass {
  public:
    DiffTimeClass (){}
    ~DiffTimeClass() {}

    void init()
    {
      ROS_INFO("Called init of class");
    }

  };

}


#endif // CALC_DIFF_TIME_H
