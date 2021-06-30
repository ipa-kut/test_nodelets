#ifndef COMP_PARAM_H
#define COMP_PARAM_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <boost/thread.hpp>
#include <cmath>

namespace calculators {

class CompParamClass{
public:
  CompParamClass(ros::NodeHandle& nh, std::string& name):
    nh_(nh),
    name_(name)
  {}
  ~CompParamClass() {}
  void init (){
    nh_.getParam("/param",param_);
    nh_.getParam("/tolerance", tolerance_);
    ROS_INFO_STREAM("Param: " << param_ << " Tol: " << tolerance_);
    control_ip_ = nh_.subscribe("control", 10, &CompParamClass::controlIpCB, this);
    float_val_sub_ = nh_.subscribe("float_val", 10, &CompParamClass::floatValCB, this);
    pub_result_ = nh_.advertise<std_msgs::Bool>("result",10);
    calculate_ = nh_.createTimer(ros::Duration(2.0), boost::bind(& CompParamClass::calculateCB, this, _1));
  }

  void controlIpCB(const std_msgs::Bool::ConstPtr& msg){
    mx_control_.lock();
    control_ = msg->data;
    mx_control_.unlock();
  }

  void floatValCB(const std_msgs::Float64::ConstPtr& msg){
    mx_control_.lock();
    float_val_ = msg->data;
    mx_control_.unlock();
  }

  void calculateCB(const ros::TimerEvent& event) {
    mx_control_.lock();
    bool control = control_;
    mx_control_.unlock();
    if(control)
    {
      mx_float_.lock();
      float float_val = float_val_;
      mx_float_.unlock();
      if (abs(float_val - param_) <= tolerance_)
      {
        ROS_INFO_STREAM("Val equal to param");
        std_msgs::Bool msg;
        msg.data = true;
        pub_result_.publish(msg);
      }
      else
      {
        ROS_INFO_STREAM("Val not equal to param");
        std_msgs::Bool msg;
        msg.data = false;
        pub_result_.publish(msg);
      }
    }
  }
private:
  ros::NodeHandle nh_;
  std::string name_;
  ros::Timer start_;
  ros::Timer stop_;
  ros::Timer calculate_;
  ros::Subscriber control_ip_;
  ros::Subscriber float_val_sub_;
  ros::Publisher pub_result_;
  boost::mutex mx_control_;
  boost::mutex mx_float_;
  bool control_;
  float float_val_;
  float param_;
  float tolerance_;

};


}


#endif // COMP_PARAM_H
