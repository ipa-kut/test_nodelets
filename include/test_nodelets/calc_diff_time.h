#ifndef CALC_DIFF_TIME_H
#define CALC_DIFF_TIME_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <boost/thread.hpp>

namespace calculators {

  class DiffTimeClass {
  public:
    DiffTimeClass (ros::NodeHandle& nh, std::string& name):
     nh_(nh),
     name_(name)
    {}
    ~DiffTimeClass() {}

    void init()
    {
      ROS_INFO("Called init of class");
      timer_ = nh_.createTimer(ros::Duration(5.0), boost::bind(& DiffTimeClass::timerCb, this, _1));
      reader_ = nh_.createTimer(ros::Duration(5.0), boost::bind(& DiffTimeClass::readerCb, this, _1));
      extern_sub_ = nh_.subscribe("extern_sub", 10, &DiffTimeClass::externSubCb, this);
      internal_sub_ = nh_.subscribe("internal_sub", 10, &DiffTimeClass::internalSubCb, this);
      internal_pub_ = nh_.advertise<std_msgs::Int32>("internal_pub", 10);
      intern_msg_.reset(new std_msgs::Int32());
    }

    void timerCb(const ros::TimerEvent& event) {
      mutex_a.lock();
      time_ = ros::Time::now();
      mutex_a.unlock();
      ROS_INFO_STREAM(name_ << " Saved time at " << time_ );
    }

    void readerCb(const ros::TimerEvent& event) {
      mutex_a.lock();
      ros::Time local_time = time_;
      mutex_a.unlock();
      ROS_INFO_STREAM(name_ << " Read time as " << time_ );
    }

    void externSubCb(const std_msgs::Int32::ConstPtr& msg) {
      ROS_INFO_STREAM(name_ << " got external value " << msg->data );
      intern_msg_->data = msg->data;
      internal_pub_.publish(intern_msg_);
      if(msg->data == 0 && timer_.hasStarted()) {
        ROS_INFO_STREAM("Stopping internal saving timer");
        timer_.stop();
      }
      if(msg->data == 1 && !timer_.hasStarted()) {
        ROS_INFO_STREAM("Starting internal saving timer");
        timer_.start();
      }
    }

    void internalSubCb(const std_msgs::Int32::ConstPtr& msg) {
      ROS_INFO_STREAM(name_ << " got internal value " << msg->data );
    }


  private:
    ros::NodeHandle nh_;
    std::string name_;
    boost::mutex mutex_a;
    ros::Timer timer_;
    ros::Timer reader_;
    ros::Time time_;
    ros::Subscriber extern_sub_;
    ros::Subscriber internal_sub_;
    ros::Publisher internal_pub_;
    boost::shared_ptr<std_msgs::Int32> intern_msg_;

  };

}


#endif // CALC_DIFF_TIME_H
