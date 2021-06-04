#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "test_nodelets/calc_diff_time.h"


namespace  calculators{
  class DiffTimeNodelet : public nodelet::Nodelet {
  public:
    DiffTimeNodelet(){}
    ~DiffTimeNodelet(){}

    virtual void onInit()
    {
      ros::NodeHandle nh = this->getPrivateNodeHandle();
      std::string name = nh.getUnresolvedNamespace();
      name = name.substr(name.find_last_of('/') + 1);

      NODELET_INFO_STREAM("Initialising nodelet... [" << name << "]");
      diff_time_.reset(new DiffTimeClass(nh, name));
      diff_time_->init();
    }
  private:
    boost::shared_ptr<DiffTimeClass> diff_time_;

  };

}

PLUGINLIB_EXPORT_CLASS(calculators::DiffTimeNodelet,
                       nodelet::Nodelet);
