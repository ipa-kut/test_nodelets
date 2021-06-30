#ifndef COMP_FLOAT_TEMPORAL_H
#define COMP_FLOAT_TEMPORAL_H


#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <boost/thread.hpp>
#include <cmath>

namespace calculators {

class CompFloatTemporalClass{
public:
  CompFloatTemporalClass(ros::NodeHandle& nh, std::string& name):
    nh_(nh),
    name_(name)
  {}
  ~CompFloatTemporalClass() {}
  void init (){
    is_started_ = is_stopped_ = is_reset_ = false;
    start_ = nh_.subscribe("start", 10, &CompFloatTemporalClass::startCB, this);
    stop_ = nh_.subscribe("stop", 10, &CompFloatTemporalClass::stopCB, this);
    reset_ = nh_.subscribe("reset", 10, &CompFloatTemporalClass::resetCB, this);
    float_val_sub_ = nh_.subscribe("float_val", 10, &CompFloatTemporalClass::floatValCB, this);
    pub_result_ = nh_.advertise<std_msgs::Float64>("result",10);
    execute_ = nh_.createTimer(ros::Duration(1.0), boost::bind(& CompFloatTemporalClass::executeCB, this, _1));
    result_msg_.reset(new std_msgs::Float64());
    ROS_INFO_STREAM("Finished init");
  }

  void startCB(const std_msgs::Bool::ConstPtr& msg){
    mx_control_.lock();
    is_started_ = msg->data;
    mx_control_.unlock();
  }

  void stopCB(const std_msgs::Bool::ConstPtr& msg){
    mx_control_.lock();
    is_stopped_ = msg->data;
    mx_control_.unlock();
  }

  void resetCB(const std_msgs::Bool::ConstPtr& msg){
    mx_control_.lock();
    is_reset_ = msg->data;
    mx_control_.unlock();
  }

  void floatValCB(const std_msgs::Float64::ConstPtr& msg){
    mx_float_.lock();
    float_val_ = static_cast<float>(msg->data);
    mx_float_.unlock();
  }

  void executeCB(const ros::TimerEvent& event) {
    is_started_ = is_stopped_ = is_reset_ = false;
    waitToStart();
    mx_float_.lock();
    float start_float = float_val_;
    mx_float_.unlock();
    ROS_INFO_STREAM("Starting float stored: " << start_float);
    ROS_INFO(" ");

    waitToStop();
    mx_float_.lock();
    float stop_float = float_val_;
    mx_float_.unlock();
    ROS_INFO_STREAM("Stopping float stored: " << stop_float);
    ROS_INFO(" ");

    float diff_float = stop_float - start_float;
    ROS_INFO_STREAM("Difference in floats: " << diff_float);
    ROS_INFO(" ");
    result_msg_->data = static_cast<double>(diff_float);
    pub_result_.publish(result_msg_);

    waitToReset();
    ROS_INFO("Restarting calculator");
    ROS_INFO("-----------------------------------------------------------");
    ROS_INFO(" ");
    ROS_INFO(" ");
  }

  void waitToStart()
  {
    ROS_INFO_STREAM("Waiting to start");
    ros::Rate rate(10);
    while(ros::ok())
    {
      mx_control_.lock();
      bool is_started = is_started_;
      mx_control_.unlock();
      if(is_started)break;
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO_STREAM("Started");
  }

  void waitToStop()
  {
    ROS_INFO_STREAM("Waiting to stop");
    ros::Rate rate(10);
    while(ros::ok())
    {
      mx_control_.lock();
      bool is_stopped = is_stopped_;
      mx_control_.unlock();
      if(is_stopped)break;
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO_STREAM("Stopped");
  }

  void waitToReset()
  {
    ROS_INFO_STREAM("Waiting to reset");
    ros::Rate rate(10);
    // Wait for reset to go high
    while(ros::ok())
    {
      mx_control_.lock();
      bool is_reset = is_reset_;
      mx_control_.unlock();
      if(is_reset_){
        // Wait for reset to go low
        while(ros::ok())
        {
          mx_control_.lock();
          is_reset = is_reset_;
          mx_control_.unlock();
          if(!is_reset) break;
        }
        break;
      }
      rate.sleep();
    }
    is_started_ = is_stopped_ = is_reset_ = false;
    ROS_INFO_STREAM("Reset");
  }


private:
  ros::NodeHandle nh_;
  std::string name_;
  ros::Timer execute_;
  ros::Subscriber start_;
  ros::Subscriber stop_;
  ros::Subscriber reset_;
  ros::Subscriber float_val_sub_;
  ros::Publisher pub_result_;
  boost::mutex mx_control_;
  boost::mutex mx_float_;
  bool is_started_;
  bool is_stopped_;
  bool is_reset_;
  float float_val_;
  boost::shared_ptr<std_msgs::Float64> result_msg_;
};
}

#endif // COMP_FLOAT_TEMPORAL_H
