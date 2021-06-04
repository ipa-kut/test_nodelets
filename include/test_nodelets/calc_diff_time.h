#ifndef CALC_DIFF_TIME_H
#define CALC_DIFF_TIME_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
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
      timer_ = nh_.createTimer(ros::Duration(1.0), boost::bind(& DiffTimeClass::timerCb, this, _1));
      reader_ = nh_.createTimer(ros::Duration(1.0), boost::bind(& DiffTimeClass::readerCb, this, _1));
    }

    void timerCb(const ros::TimerEvent& event) {
      mutex_a.lock();
      time_ = ros::Time::now();
      mutex_a.unlock();
      ROS_INFO_STREAM("Saved time at " << time_ );
    }

    void readerCb(const ros::TimerEvent& event) {
      mutex_a.lock();
      ros::Time local_time = time_;
      mutex_a.unlock();
      ROS_INFO_STREAM("Read time as " << time_ );
    }


  private:
    ros::NodeHandle nh_;
    std::string name_;
    boost::mutex mutex_a;
    ros::Timer timer_;
    ros::Timer reader_;
    ros::Time time_;

  };

}


#endif // CALC_DIFF_TIME_H
