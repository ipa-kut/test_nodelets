#ifndef CALC_DIFF_INT_HPP
#define CALC_DIFF_INT_HPP

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <boost/thread.hpp>
#include <cmath>
#include <vector>

#include <ambs_base_calculator/ambs_base_calculator.hpp>
#include <ambs_base_interface/ambs_base_interface.hpp>

namespace calculators {

class CalcDiffInt : public ambs_base::AMBSBaseCalculator
{
public:
  CalcDiffInt() {};
  ~CalcDiffInt() {};
  CalcDiffInt(ros::NodeHandle& nh, std::string& name):
    nh_(nh),
    name_(name),
    ambs_base::AMBSBaseCalculator (nh)
  {}

  void init (){
    ROS_INFO_STREAM(name_ <<" : Init class");

    standard_control_.init(nh_);
    std::vector<std::string> int_inputs;
    int_inputs.push_back("/in_int");
    std::vector<std::string> int_outputs;
    int_outputs.push_back("/out_int");
    int_interface_.init(int_inputs, int_outputs, nh_);

    startCalculator();
  }

  void executeCB(const ros::TimerEvent& event) {
      standard_control_.waitForStart();
      int start_int = int_interface_.getPortMsg("/in_int").data;
      ROS_INFO_STREAM("Starting int stored: " << start_int);
      ROS_INFO(" ");

      standard_control_.waitForStop();
      int stop_int = int_interface_.getPortMsg("/in_int").data;
      ROS_INFO_STREAM("Stopping int stored: " << stop_int);
      ROS_INFO(" ");

      int diff_int = stop_int - start_int;
      ROS_INFO_STREAM("Difference in int: " << diff_int);
      ROS_INFO(" ");
      result_msg_.data = diff_int;
      int_interface_.publishMsgOnPort("/out_int", result_msg_);

      standard_control_.waitForReset();
      ROS_INFO("Restarting calculator");
      ROS_INFO("-----------------------------------------------------------");
      ROS_INFO(" ");
      ROS_INFO(" ");
    }

private:
  ros::NodeHandle nh_;
  std::string name_;
  ros::Timer execute_;
  ambs_base::AMBSTemplatedInterface<std_msgs::Int32> int_interface_;
  std_msgs::Int32 result_msg_;
};

}
#endif // CALC_DIFF_INT_HPP
